
/////////////////////////////////////////////////////////
// module adds .PCA9685Servos(opts) to pigpio.
// opts like:
//{
//    servo_max_us: DEFAULT_SERVO_MAX_US, // 500
//    servo_min_us: DEFAULT_SERVO_MIN_US, // 2500
//    cycle_time_us: DEFAULT_CYCLE_TIME_US, // 20,000
//    debug: 0,
//}
// Usage:
// after pigpio connect, 
// let opts = {};
// let servos = pigpio.PCA9685Servos(opts);
// servos.setServoLimits(0, 0.2, 0.8);
// let setPosn = await servos.setServo(0, 0.5);
//
// functions:
// servos.setServoLimits(index, min, max); // set min/mas lmits for a servo (0-1)
// let posn = await servos.setServo(index, posn); // set a servo to a posn (0-1)|-1, return limited posn or -1.  -1 indicates stop pwm for that servo
// await servos.sleep(); // stop the clock, sleep device
// await servos.wake(); // start the clock, wake device
// await servos.clearServos(); // stop pwm for all servos
// await servos.allServos(posn); // set all servos to posn (0.0-1.0)|-1  posn for each will be limited by user limits if set
// await servos.close(); // close the i2c handle on pigpio.
// await servos.init(); // re-initialise, e.g. if pigpio reconnected.  tries to close existing if present.
// await servos.wait_ms(ms); // simple awaitable delay
/////////////////////////////////////////////////////////

function addPCA9685Servos(pigpio_instance, opts) {
    let pigpio = pigpio_instance;
    pigpio_instance.PCA9685Servos = async function (opts) {
        const DEFAULT_CYCLE_TIME_US = 1000000 / 50 // default to 50Hz -> 20,000uS
        const DEFAULT_STEP_TIME_US = 5 // gives 400 steps for typical servo range
        const DEFAULT_SERVO_MIN_US = 500 // Be aware that many cheap servos get very annoyed
        const DEFAULT_SERVO_MAX_US = 2500// by getting pushed too far. Use the min/max

        const MAX_SERVOS = 16;
        const I2C_BUS = 1;
        const DEFAULT_PCA_ADDR = 0x40;

        const MODE1 = 0x00			//Mode  register  1
        const MODE2 = 0x01			//Mode  register  2
        const SUBADR1 = 0x02		//I2C-bus subaddress 1
        const SUBADR2 = 0x03		//I2C-bus subaddress 2
        const SUBADR3 = 0x04		//I2C-bus subaddress 3
        const ALLCALLADR = 0x05     //LED All Call I2C-bus address
        const LED0 = 0x6			//LED0 start register
        const LED0_ON_L = 0x6		//LED0 output and brightness control byte 0
        const LED0_ON_H = 0x7		//LED0 output and brightness control byte 1
        const LED0_OFF_L = 0x8		//LED0 output and brightness control byte 2
        const LED0_OFF_H = 0x9		//LED0 output and brightness control byte 3
        const LED_MULTIPLYER = 4	// For the other 15 channels
        const ALLLED_ON_L = 0xFA    //load all the LEDn_ON registers, byte 0 (turn 0-7 channels on)
        const ALLLED_ON_H = 0xFB	//load all the LEDn_ON registers, byte 1 (turn 8-15 channels on)
        const ALLLED_OFF_L = 0xFC	//load all the LEDn_OFF registers, byte 0 (turn 0-7 channels off)
        const ALLLED_OFF_H = 0xFD	//load all the LEDn_OFF registers, byte 1 (turn 8-15 channels off)
        const PRE_SCALE = 0xFE		//prescaler for output frequency
        const CLOCK_FREQ = 25000000.0 //25MHz default osc clock
        const BUFFER_SIZE = 0x08  //1 byte buffer
        // MODE1 reg flags
        const RESTART = 0x80
        const EXTCLK = 0x40
        const AI = 0x20
        const SLEEP = 0x10
        const NOTSLEEP = 0x7F
        const SUB1 = 0x8
        const SUB2 = 0x4
        const SUB3 = 0x2
        const ALLCALL = 0x1
        // MODE2 reg flags
        const INVRT = 0x10
        const OCH = 0x8
        const OUTDRV = 0x4
        //const OUTNE // doesn't matter here


        opts = opts || {};
        let defaultopts = {
            bus: I2C_BUS,
            i2c_address: DEFAULT_PCA_ADDR,
            servo_max_us: DEFAULT_SERVO_MAX_US,
            servo_min_us: DEFAULT_SERVO_MIN_US,
            cycle_time_us: DEFAULT_CYCLE_TIME_US,
            debug: 0,
        };

        // duplicate opts structure 
        opts = Object.assign({}, defaultopts, opts);

        let PCA9685 = {
            opts: opts,
            pca: undefined,
            servowidth: [], // as a value 0-1
            servostartus: [], // in us
            servoMinMax: {}, // enforced min/max values for each servo

            wait_ms: function (millisec) {
                return new Promise(resolve => {
                    setTimeout(() => { resolve('') }, millisec);
                });
            },

            init: async function () {
                try {
                    // if re-initialising, try to close first
                    if (this.pca){
                        await this.close();
                    }
                    // connect to the PCA9685 via i2c
                    this.pca = await pigpio.i2c(this.opts.bus, this.opts.i2c_address);

                    // set all pwm regs to zero
                    await this.clearServos();

                    // initialise the PCA; write config byte to reg 0
                    // See PCA9685.pdf 7.3.1
                    // exactly what is best here is a bit arguable. I see 0x20 or 0x21 or 0 used variously
                    await this.pca.writeByteData(MODE1, AI | ALLCALL);

                    // maybe we should set some flags in MODE2 as well?
                    // 0xC is used in at least one python based driver
                    await this.pca.writeByteData(MODE2, /*OCH | */ OUTDRV);
                    // we have to wait for at least 500uS after setting the SLEEP flag to 0
                    await this.wait_ms(10);
                    this.cycle_time_us = this._calculateTimerSettings(this.opts.cycle_time_us);
                    this._init_servo_starts();
                    await this._setPwmFreq();
                    // make all servos unpowered at start.
                } catch (e) {
                    console.error('i2c init error', e);
                }
            },

            close: async function () {
                try {
                    if (this.pca) {
                        await this.clearServos();
                        await this.sleep(); // stop the clock
                        await this.pca.close();
                        if (this.move_interval){
                            clearInterval(this.move_interval);
                            this.move_interval = null;
                        }
                        this.pca = undefined;
                    }
                } catch(e){
                    this.pca = undefined;
                }
            },

            _init_servo_starts: function () {
                let us_per_servo = this.cycle_time_us / MAX_SERVOS;
                let curr_us = 10;

                /* set the servo start ticks to spread out the current draw ? */
                for (let servo = 0; servo < MAX_SERVOS; servo++) {
                    this.servowidth.push(0);
                    if (curr_us + this.opts.servo_max_us >= this.opts.cycle_time_us) {
                        curr_us = us_per_servo / 3;
                    }
                    this.servostartus.push(curr_us);
                    curr_us += us_per_servo;
                }
            },

            _calculateTimerSettings: function (cycle_time) {
                let freq = 1000000 / cycle_time;
                this.timer_prescale = (CLOCK_FREQ / 4096 / freq) - 1;
                //console.log("Setting prescale value to: "+ timer_prescale);
                //console.log("Actual frequency: "+ (CLOCK_FREQ / 4096.0) / (timer_prescale + 1));
                // adjust the global cycle time to reflect reality
                let cycle_time_us = 1000000 * (this.timer_prescale + 1) / (CLOCK_FREQ / 4096.0);
                if (this.opts.debug) {
                    let steps = ((((this.opts.servo_max_us - this.opts.servo_min_us) / cycle_time_us) * 4096) >> 0);
                    console.log("cycle time: " + cycle_time_us + "ms");
                    console.log("servo_min_us " + this.opts.servo_min_us + " servo_max_us " + this.opts.servo_max_us +
                        " travel " + (this.opts.servo_max_us - this.opts.servo_min_us) + "us");
                    console.log("steps full travel " + ((((this.opts.servo_max_us - this.opts.servo_min_us) / cycle_time_us) * 4096) >> 0));
                    console.log("servo resolution ~" + (((10000 / steps) >> 0) / 100) + "%");
                }
                return cycle_time_us;
            },

            _setPwmFreq: async function () {
                if (this.pca) {
                    let oldmode = await this.pca.readByteData(MODE1);
                    let newmode = (oldmode & NOTSLEEP) | SLEEP;    //sleep
                    await this.pca.writeByteData(MODE1, newmode);        // go to sleep
                    await this.pca.writeByteData(PRE_SCALE, this.timer_prescale);
                    await this.pca.writeByteData(MODE1, oldmode);
                    await this.wait_ms(10);
                    await this.pca.writeByteData(MODE1, oldmode | RESTART);
                }
            },

            // stops the clock, and hence output
            sleep: async function () {
                if (this.pca) {
                    try {
                        let oldmode = await this.pca.readByteData(this.pca, MODE1);
                        let newmode = (oldmode & NOTSLEEP) | SLEEP;    //sleep
                        await this.pca.writeByteData(MODE1, newmode);        // go to sleep
                    } catch (e) {
                        console.error(e);
                    }
                }
            },

            // start the clock, and hence output
            wake: async function () {
                if (this.pca) {
                    try {
                        let oldmode = await this.pca.readByteData(this.pca, MODE1);
                        let newmode = (oldmode & NOTSLEEP);    // wake
                        await this.pca.writeByteData(MODE1, newmode);
                    } catch (e) {
                        console.error(e);
                    }
                }
            },

            // read back the regs for one channel
            read_servo: async function (servo) {
                if ((servo < 0) || (servo >= MAX_SERVOS)) return {on:0,off:0};
                if (this.pca) {
                    let b = await this.pca.readI2cBlockData(LED0_ON_L + LED_MULTIPLYER * servo, 4);
                    let on = b[0] | (b[1] << 8);
                    let off = b[2] | (b[3] << 8);
                    return { on, off };
                }
            },

            // set a servo position, width 0-1
            setServo: async function (servo, width) {
                if ((servo < 0) || (servo >= MAX_SERVOS)) return -2;
                if (this.pca) {
                    try {
                        if (width === -1) {
                            let data = [0, 0, 0, 0];
                            await this.pca.writeI2cBlockData(LED0_ON_L + LED_MULTIPLYER * servo, data);
                            return -1;
                        } else {
                            // servoMinMax is a user set limit on servo travel, in values 0-1
                            if (this.servoMinMax[servo]) {
                                if (width < this.servoMinMax[servo].min) {
                                    width = this.servoMinMax[servo].min;
                                }
                                if (width > this.servoMinMax[servo].max) {
                                    width = this.servoMinMax[servo].max;
                                }
                            } else {
                                if (width < 0) width = 0;
                                if (width > 1) width = 1;
                            }

                            this.servowidth[servo] = width;
                            //console.log( `set servo[${servo}]=${width*100}`);
                            // set this servo to start at the servostart tick and stay on for width ticks
                            let range = (this.opts.servo_max_us - this.opts.servo_min_us);
                            let val = this.servowidth[servo] * range;
                            let dur = this.opts.servo_min_us + val;

                            let on_us = this.servostartus[servo];
                            let off_us = on_us + dur;

                            let on_value = ((on_us / this.opts.cycle_time_us) * 4096) >> 0;
                            let off_value = ((off_us / this.opts.cycle_time_us) * 4096) >> 0;

                            //console.log(`rangeus ${range} valus ${val} durus ${dur} offval ${off_value}`);
                            if (off_value > 4095) off_value = 4095;
                            on_value = on_value >> 0;
                            off_value = off_value >> 0;

                            if (off_value < on_value){
                                console.error(`off ${off_value} is less than on ${on_value}`);
                            }

                            let data = [on_value & 0xff, (on_value >> 8) & 0xff, off_value & 0xff, (off_value >> 8) & 0xff];

                            // tihs results in later errors, and does not work
                            await this.pca.writeI2cBlockData(LED0_ON_L + LED_MULTIPLYER * servo, data);
                            if (this.opts.debug && 0) {
                                let res = await this.read_servo(servo);
                                let diff = (res.off - res.on) * this.opts.cycle_time_us / 4096;
                                let percent = (((((diff - this.opts.servo_min_us) / range) * 100) * 10) >> 0) / 10;
                                console.log('read servo ' + servo, res, '' + (diff >> 0) + 'us', percent + '%');
                            }
                            // return the value we actually set.
                            return this.servowidth[servo];
                        }
                    } catch (e) {
                        console.log('set servo exception', e);
                    }
                }
            },

            setServoLimits: function (servo, min, max) {
                if ((servo < 0) || (servo >= MAX_SERVOS)) return;
                min = min || 0;
                max = max || 1;
                if (min < 0) min = 0;
                if (max > 1) max = 1;
                this.servoMinMax[servo] = { min, max };
                if (this.opts.debug) console.log('set '+servo+ ' minmax to ',this.servoMinMax[servo]);
            },

            // be sure to await this function - it returns a promise.
            // scaledposn is 0-1 representing max-min for this servo.
            setServoScaled: function (servo, scaledposn) {
                let posn = scaledposn;
                if (this.servoMinMax[servo]){
                    let range = this.servoMinMax[servo].max - this.servoMinMax[servo].min;
                    posn = (posn*range) + this.servoMinMax[servo].min;
                }
                return this.setServo(servo, posn);
            },

            clearServos: async function () {
                for (let i = 0; i < MAX_SERVOS; i++) {
                    await this.setServo(i, -1);
                }
            },

            // although we could use the destinct fucntion to set all channels,
            // then there would be no lmits.
            allServos: async function (val) {
                for (let i = 0; i < MAX_SERVOS; i++) {
                    await this.setServo(i, val);
                }
            },



            //////////////////////////////////////////////////////
            // slow move routines.

            // NOT ASYNC
            // scaledposn is 0-1 representing max-min for this servo.
            moveToScaled: function (servo, scaledposn, duration_ms) {
                let posn = scaledposn;
                if (this.servoMinMax[servo]){
                    let range = this.servoMinMax[servo].max - this.servoMinMax[servo].min;
                    posn = (posn*range) + this.servoMinMax[servo].min;
                }
                if (this.opts.debug) console.log('moveToScaled '+servo+ ' scaledposn '+scaledposn+' -> posn '+posn);

                return this.moveTo(servo, posn, duration_ms);
            },

            targets:{},
            move_interval:null,
            // set a target posn and a duration to get there.
            // NOT ASYNC
            moveTo: function(servo, target_posn, duration_ms){
                if (target_posn > 1) target_posn = 1;
                if (target_posn < 0) target_posn = 0;

                let start_time = (new Date()).valueOf();
                let start_posn = this.servowidth[servo];
                let target_time = start_time + duration_ms;
                this.targets[servo] = {
                    start_posn,
                    start_time,
                    target_posn,
                    target_time,
                };
                if (!this.move_interval){
                    this.move_interval = setInterval(this.moveSome.bind(this), 40);
                }
            },

            moveSome: async function(){
                let targets = Object.keys(this.targets);
                let now = (new Date()).valueOf();
                for (let i = 0; i < targets.length; i++){
                    let done = false;
                    let t = this.targets[targets[i]];
                    let posn = (t.target_posn - t.start_posn)*((now - t.start_time)/(t.target_time - t.start_time)) + t.start_posn;
                    if (t.target_posn > t.start_posn){
                        if (posn >= t.target_posn){
                            posn = t.target_posn;
                            done = true;
                            if (this.opts.debug) console.log(targets[i]+' got to '+posn);
                        }
                    } else {
                        if (posn <= t.target_posn){
                            posn = t.target_posn;
                            done = true;
                            if (this.opts.debug) console.log(targets[i]+' got to '+posn);
                        }
                    }
                    this.setServo(+targets[i], posn);
                    if (done) delete this.targets[targets[i]];
                    else {
                        if (this.opts.debug) console.log(targets[i]+' move to '+posn+' target '+t.target_posn);
                    }
                }
            },
        };

        await PCA9685.init();
        return PCA9685;
    };
}


module.exports = addPCA9685Servos;

//////////////////////////////////////////////////////////////////////////////////
// Example covering all aspects of servo control
//
// Usage:
// node PCA9685-servo-test.js <op>? <host>?
// op = 
// host = ip or domain name where pigpio is exposing port 8888 - defaults to 127.0.0.1, 
// e.g. from the examples folder:
// node PCA9685-servo-test.js poll 192.168.1.185
//
//////////////////////////////////////////////////////////////////////////////////

let host = '127.0.0.1';

// this must be the branch modified to include i2c
// https://github.com/btsimonh/pigpio-client/tree/Addmorei2c
const pigpioClient = require('pigpio-client');
let addPCA9685Servos = require('../index.js');

// comment out to stop debug
//proc.env['DEBUG'] = 'pigpio';
//proc.env['PIGPIO'] = '1';

// simple ansi screen stuff.
let CLS =  '\033[2J';
let BLUE="\033[0;34m";
let RED="\033[0;31m";
let LIGHT_RED="\033[1;31m";
let WHITE="\033[1;37m";
let NO_COLOUR="\033[0m";

let op = 'wiggle12';

// set to 1 for display of avg values at start.
let debug = 0;

const args = process.argv;
if (args.length > 2) op = args[2]; 
if (args.length > 3) host = args[3];

// Connect to the Rpi...
console.log('trying to connect to '+host);

let opts = {
    host: host, // the host it will try to connect to port 8888 on....
    //port: 8888, // default is 8888
    timeout: 3, // retry timer in s?
};

let pigpio = pigpioClient.pigpio(opts);  

const ready = new Promise((resolve, reject) => {
    pigpio.once('connected', resolve);
    pigpio.once('error', reject);
});
  
ready.then(async (info) => {
    // display information on pigpio and connection status
    //console.log(JSON.stringify(info,null,2));
    addPCA9685Servos(pigpio);
    // must await this, as it opens the i2c and initialises the device
    let servos = await pigpio.PCA9685Servos({debug:1});

    let index = 13;
    servos.setServoLimits(index, 0.2, 0.8);
    switch (op){
        case 'wiggle12':{
            let posn = 0.5;
            for (let i = 0; i < 10; i++){
                //let limited = await servos.setServo(index, posn);
                //console.log('set servo to '+limited);
                posn = i & 1;

                servos.moveToScaled(index, posn, 2000);
                await servos.wait_ms(3000);
            }
        } break;
    }
    await servos.close();
    await pigpio.end();
});


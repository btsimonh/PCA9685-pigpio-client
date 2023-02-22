# PCA9685-pigpio-client
PCA9685 servo driver based on pigpio-client

module adds .PCA9685Servos(opts) to pigpio.

## install

```
npm install --save github:btsimonh/PCA9685-pigpio-client
```

this depends upon github:btsimonh/pigpio-client#Addmorei2c


## Usage:

let pca = require('PCA9685-pigpio-client');

pca(pigpio);


after pigpio connect, 
opts like:
```
{
    servo_max_us: DEFAULT_SERVO_MAX_US, // 500
    servo_min_us: DEFAULT_SERVO_MIN_US, // 2500
    cycle_time_us: DEFAULT_CYCLE_TIME_US, // 20,000
    debug: 0,
}
```

```
let opts = {};
let servos = pigpio.PCA9685Servos(opts);
servos.setServoLimits(0, 0.2, 0.8);
let setPosn = await servos.setServo(0, 0.5);
```

## functions:
```
servos.setServoLimits(index, min, max); // set min/mas lmits for a servo (0-1)
let posn = await servos.setServo(index, posn); // set a servo to a posn (0-1)|-1, return limited posn or -1.  -1 indicates stop pwm for that servo
await servos.sleep(); // stop the clock, sleep device
await servos.wake(); // start the clock, wake device
await servos.clearServos(); // stop pwm for all servos
await servos.allServos(posn); // set all servos to posn (0.0-1.0)|-1  posn for each will be limited by user limits if set
await servos.close(); // close the i2c handle on pigpio.
await servos.init(); // re-initialise, e.g. if pigpio reconnected.  tries to close existing if present.
await servos.wait_ms(ms); // simple awaitable delay
await servos.setServoScaled(servo, scaledposn); // set a posn 0-1 which will be scales to min-max as set by setServoLimits
```

### movement functions
```
moveTo(servo, target_posn, duration_ms); // NOT ASYNC.  move the servo to a new position over a period (starts a 40ms timer)
moveToScaled(servo, scaled_target_posn, duration_ms); // NOT ASYNC.  move the servo to a new position over a period (starts a 40ms timer) - scaled so 0-1 -> min-max as set.
```


# Acknowledgments

It was made possible because of the brilliant work by the writers of [piopiod](https://abyz.me.uk/rpi/pigpio/pigpiod.html), and the implementaiton of [pigpio-client](https://github.com/guymcswain/pigpio-client) by @guymcswain (hopefully he'll merge my i2c enhancements soon...).  

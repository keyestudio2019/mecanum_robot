/**
 * use for RGB-LED
 */
enum COLOR {
    red,
    green,
    blue,
    white,
    black
}
/**
  * Pre-Defined LED colours
  */
enum vColors {
    //% block=red
    Red = 0xff0000,
    //% block=orange
    Orange = 0xffa500,
    //% block=yellow
    Yellow = 0xffff00,
    //% block=green
    Green = 0x00ff00,
    //% block=blue
    Blue = 0x0000ff,
    //% block=indigo
    Indigo = 0x4b0082,
    //% block=violet
    Violet = 0x8a2be2,
    //% block=purple
    Purple = 0xff00ff,
    //% block=white
    White = 0xffffff,
    //% block=black
    Black = 0x000000
}
/**
 * use for control motor
 */
enum DIR {
    Run_forward = 0,
    Run_back = 1,
    Turn_Left = 2,
    Turn_Right = 3
}
enum LR {
    Upper_left = 0,
    Lower_left = 1,
    Upper_right = 2,
    Lower_right = 3,
}
enum MotorState {
    stop = 0,
    brake = 1
}
enum MD {
    Forward = 0,
    Back = 1
}

enum LT {
    Left,
    Center,
    Right
}

enum LED_STATE {
    HIGH = 4095,
    LOW = 0
}

//% color="#ff6800" icon="\uf1b9" weight=15
//% groups="['Motor', 'RGB-led', 'Neo-pixel', 'Sensor', 'Tone']"
namespace mecanumRobot {
    /**
     * use for control PCA9685
     */

    const PCA9685_ADDRESS = 0x47;   //device address
    const MODE1 = 0x00;
    let initI2C = false;
    let _i2cError = 0;
    let SERVOS = 0x06; // first servo address for start byte low
    let servoTarget: number[] = [];
    let servoActual: number[] = [];
    let servoCancel: boolean[] = [];
    //const MODE2 = 0x01;
    //const SUBADR1 = 0x02;
    //const SUBADR2 = 0x03;
    //const SUBADR3 = 0x04;
    const PRESCALE = 0xFE;
    const LED0_ON_L = 0x06;
    const LED0_ON_H = 0x07;
    const LED0_OFF_L = 0x08;
    //const LED0_OFF_H = 0x09;
    //const ALL_LED_ON_L = 0xFA;
    //const ALL_LED_ON_H = 0xFB;
    //const ALL_LED_OFF_L = 0xFC;
    //const ALL_LED_OFF_H = 0xFD;

    let PCA9685_Initialized = false
    // initialise the servo driver and the offset array values
    function initPCA(): void {

        let i2cData = pins.createBuffer(2);
        initI2C = true;

        i2cData[0] = 0;		// Mode 1 register
        i2cData[1] = 0x10;	// put to sleep
        pins.i2cWriteBuffer(PCA9685_ADDRESS, i2cData, false);

        i2cData[0] = 0xFE;	// Prescale register
        i2cData[1] = 101;	// set to 60 Hz
        pins.i2cWriteBuffer(PCA9685_ADDRESS, i2cData, false);

        i2cData[0] = 0;		// Mode 1 register
        i2cData[1] = 0x81;	// Wake up
        pins.i2cWriteBuffer(PCA9685_ADDRESS, i2cData, false);

        for (let servo = 0; servo < 16; servo++) {
            i2cData[0] = SERVOS + servo * 4 + 0;	// Servo register
            i2cData[1] = 0x00;			// low byte start - always 0
            _i2cError = pins.i2cWriteBuffer(PCA9685_ADDRESS, i2cData, false);

            i2cData[0] = SERVOS + servo * 4 + 1;	// Servo register
            i2cData[1] = 0x00;			// high byte start - always 0
            pins.i2cWriteBuffer(PCA9685_ADDRESS, i2cData, false);

            servoTarget[servo] = 0;
            servoActual[servo] = 0;
            servoCancel[servo] = false;
        }
    }
    

    function i2cRead(addr: number, reg: number) {
        pins.i2cWriteNumber(addr, reg, NumberFormat.UInt8BE);
        let val = pins.i2cReadNumber(addr, NumberFormat.UInt8BE);
        return val;
    }

    function i2cWrite(PCA9685_ADDRESS: number, reg: number, value: number) {
        let buf = pins.createBuffer(2)
        buf[0] = reg
        buf[1] = value
        pins.i2cWriteBuffer(PCA9685_ADDRESS, buf)
    }

    function setFreq(freq: number): void {
        // Constrain the frequency
        let prescaleval = 25000000;
        prescaleval /= 4096;
        prescaleval /= freq;
        prescaleval -= 1;
        let prescale = prescaleval; //Math.Floor(prescaleval + 0.5);
        let oldmode = i2cRead(PCA9685_ADDRESS, MODE1);
        let newmode = (oldmode & 0x7F) | 0x10; // sleep
        i2cWrite(PCA9685_ADDRESS, MODE1, newmode); // go to sleep
        i2cWrite(PCA9685_ADDRESS, PRESCALE, prescale); // set the prescaler
        i2cWrite(PCA9685_ADDRESS, MODE1, oldmode);
        control.waitMicros(5000);
        i2cWrite(PCA9685_ADDRESS, MODE1, oldmode | 0xa1);
    }

    function setPwm(channel: number, on: number, off: number): void {
        let buf = pins.createBuffer(5);
        buf[0] = LED0_ON_L + 4 * channel;
        buf[1] = on & 0xff;
        buf[2] = (on >> 8) & 0xff;
        buf[3] = off & 0xff;
        buf[4] = (off >> 8) & 0xff;
        pins.i2cWriteBuffer(PCA9685_ADDRESS, buf);
    }

    function init_PCA9685(): void {
        i2cWrite(PCA9685_ADDRESS, MODE1, 0x00);  //initialize the mode register 1
        setFreq(50);   //20ms
        for (let idx = 0; idx < 16; idx++) {
            setPwm(idx, 0, 0);
        }
        PCA9685_Initialized = true;
    }

    
    /**
     * set cat state
     */
    //% block="car $sta"
    //% group="Motor" weight=98
    export function state(sta: MotorState) {
        //if (!PCA9685_Initialized) {
        //init_PCA9685();
        //}

        if (sta == 0) {           //stop
            setPwm(0, 0, 4095);  //control speed : 0---4095
            setPwm(1, 0, 0);
            setPwm(2, 0, 0);
            setPwm(5, 0, 4095);  //control speed : 0---4095
            setPwm(4, 0, 0);
            setPwm(3, 0, 0);

            setPwm(6, 0, 4095);  //control speed : 0---4095
            setPwm(7, 0, 0);
            setPwm(8, 0, 0);
            setPwm(11, 0, 4095);  //control speed : 0---4095
            setPwm(10, 0, 0);
            setPwm(9, 0, 0);
        }

        if (sta == 1) {           //brake
            setPwm(0, 0, 0);  //control speed : 0---4095
            //setPwm(1, 0, 4095);
            //setPwm(2, 0, 4095);
            setPwm(5, 0, 0);  //control speed : 0---4095
            //setPwm(4, 0, 4095);
            //setPwm(3, 0, 4095);
            setPwm(6, 0, 0);  //control speed : 0---4095
            setPwm(11, 0, 0);  //control speed : 0---4095
        }
    }
    /**
     * set speed of motor
     */
    //% block="$M motor run $D speed: $speed \\%"
    //% speed.min=0 speed.max=100
    //% group="Motor" weight=97
    export function Motor(M: LR, D: MD, speed: number) {
        if (!PCA9685_Initialized) {
            init_PCA9685();
        }
        let speed_value = Math.map(speed, 0, 100, 0, 4095);
        if (M == 2 && D == 1) {
            setPwm(0, 0, speed_value);  //control speed : 0---4095
            setPwm(1, 0, 0);
            setPwm(2, 0, 4095);
        }
        if (M == 2 && D == 0) {
            setPwm(0, 0, speed_value);  //control speed : 0---4095
            setPwm(1, 0, 4095);
            setPwm(2, 0, 0);
        }

        if (M == 0 && D == 0) {
            setPwm(5, 0, speed_value);  //control speed : 0---4095
            setPwm(4, 0, 0);
            setPwm(3, 0, 4095);
        }
        if (M == 0 && D == 1) {
            setPwm(5, 0, speed_value);  //control speed : 0---4095
            setPwm(4, 0, 4095);
            setPwm(3, 0, 0);
        }

        if (M == 3 && D == 1) {
            setPwm(6, 0, speed_value);  //control speed : 0---4095
            setPwm(7, 0, 0);
            setPwm(8, 0, 4095);
        }
        if (M == 3 && D == 0) {
            setPwm(6, 0, speed_value);  //control speed : 0---4095
            setPwm(7, 0, 4095);
            setPwm(8, 0, 0);
        }

        if (M == 1 && D == 0) {
            setPwm(11, 0, speed_value);  //control speed : 0---4095
            setPwm(10, 0, 0);
            setPwm(9, 0, 4095);
        }
        if (M == 1 && D == 1) {
            setPwm(11, 0, speed_value);  //control speed : 0---4095
            setPwm(10, 0, 4095);
            setPwm(9, 0, 0);
        }

    }

    /**
      * Initialise all servos to Angle=0
      */
    //% blockId="centreServos"
    //% block="centre all servos"
    //% subcategory=Servos
    export function centreServos(): void {
        for (let i = 0; i < 16; i++)
            setServo(i, 0);
    }

    /**
      * Set Servo Position by Angle
      * @param servo Servo number (0 to 15)
      * @param angle degrees to turn servo (-90 to +90)
      */
    //% blockId="an_setServo" block="set servo %servo| to angle %angle"
    //% weight=70
    //% angle.min=-90 angle.max.max=90
    //% subcategory=Servos
    export function setServo(servo: number, angle: number): void {
        setServoRaw(servo, angle);
        servoTarget[servo] = angle;
    }

    function setServoRaw(servo: number, angle: number): void {
        if (initI2C == false) {
            initPCA();
        }
        // two bytes need setting for start and stop positions of the servo
        // servos start at SERVOS (0x06) and are then consecutive blocks of 4 bytes
        // the start position (always 0x00) is set during init for all servos

        let i2cData = pins.createBuffer(2);
        let start = 0;
        angle = Math.max(Math.min(90, angle), -90);
        let stop = 369 + angle * 223 / 90;

        i2cData[0] = SERVOS + servo * 4 + 2;	// Servo register
        i2cData[1] = (stop & 0xff);		// low byte stop
        pins.i2cWriteBuffer(PCA9685_ADDRESS, i2cData, false);

        i2cData[0] = SERVOS + servo * 4 + 3;	// Servo register
        i2cData[1] = (stop >> 8);		// high byte stop
        pins.i2cWriteBuffer(PCA9685_ADDRESS, i2cData, false);
        servoActual[servo] = angle;
    }

    /**
      * Move Servo to Target Position at selected Speed
      * @param servo Servo number (0 to 15)
      * @param angle degrees to turn to (-90 to +90)
      * @param speed degrees per second to move (1 to 1000) eg: 60
      */
    //% blockId="moveServo" block="move servo %servo| to angle %angle| at speed %speed| degrees/sec"
    //% weight=70
    //% angle.min=-90 angle.max.max=90
    //% speed.min=1 speed.max=1000
    //% subcategory=Servos
    export function moveServo(servo: number, angle: number, speed: number): void {
        let step = 1;
        let delay = 10; // 10ms delay between steps
        if (servoTarget[servo] != servoActual[servo])   // cancel any existing movement on this servo?
        {
            servoCancel[servo] = true;
            while (servoCancel[servo])
                basic.pause(1);  // yield
        }
        angle = Math.max(Math.min(90, angle), -90);
        speed = Math.max(Math.min(1000, speed), 1);
        delay = Math.round(1000 / speed);
        servoTarget[servo] = angle;
        if (angle < servoActual[servo])
            step = -1;
        control.inBackground(() => {
            while (servoActual[servo] != servoTarget[servo]) {
                if (servoCancel[servo]) {
                    servoCancel[servo] = false;
                    break;
                }
                setServoRaw(servo, servoActual[servo] + step);
                basic.pause(delay);
            }
        })
    }

    /**
      * Get Servo Current Actual Position
      * @param servo Servo number (0 to 15)
      */
    //% blockId="getServoActual" block="servo %servo| actual position"
    //% weight=10
    //% subcategory=Servos
    export function getServoActual(servo: number): number {
        return servoActual[servo];
    }

    /**
      * Get Servo Target Position
      * @param servo Servo number (0 to 15)
      */
    //% blockId="getServoTarget" block="servo %servo| target position"
    //% weight=8
    //% subcategory=Servos
    export function getServoTarget(servo: number): number {
        return servoTarget[servo];
    }

    /**
      * Check if servo has reached target
      * @param servo Servo number (0 to 15)
      */
    //% blockId="isServoDone" block="servo %servo| is complete"
    //% weight=5
    //% subcategory=Servos
    export function isServoDone(servo: number): boolean {
        return servoTarget[servo] == servoActual[servo];
    }

    /**
      * Wait until servo has reached target position
      * @param servo Servo number (0 to 15)
      */
    //% blockId="waitServo" block="wait for servo %servo"
    //% weight=5
    //% subcategory=Servos
    export function waitServo(servo: number): void {
        while (servoActual[servo] != servoTarget[servo]) // what if nothing is changing these values?
            basic.pause(10);
    }
    
    /////////////////////////////////////////////////////
    /**
     * set rgb-led brightness
     */
    let L_brightness = 4095;  //control the rgb-led brightness
    //% block="LED brightness $br"
    //% br.min=0 br.max=255
    //% group="RGB-led" weight=79
    export function LED_brightness(br: number) {
        if (!PCA9685_Initialized) {
            init_PCA9685();
        }
        L_brightness = Math.map(br, 0, 255, 0, 4095);
    }
    /**
     * set the rgb-led color via the color card
     */
    //% block="set $RgbLed RGBled $col"
    //% group="RGB-led" weight=78
    export function Led(RgbLed: LR, col: COLOR) {
        if (!PCA9685_Initialized) {
            init_PCA9685();
        }

        if (RgbLed == 0) {    //left side RGB_LED
            setPwm(9, 0, 0);
            setPwm(10, 0, 0);
            setPwm(11, 0, 0);
            if (col == COLOR.red) {
                setPwm(9, 0, L_brightness);
            }
            if (col == COLOR.green) {
                setPwm(10, 0, L_brightness);
            }
            if (col == COLOR.blue) {
                setPwm(11, 0, L_brightness);
            }
            if (col == COLOR.white) {
                setPwm(9, 0, L_brightness);
                setPwm(10, 0, L_brightness);
                setPwm(11, 0, L_brightness);
            }
            if (col == COLOR.black) {
            }
        }

        if (RgbLed == 1) {    //right side RGB_LED
            setPwm(6, 0, 0);
            setPwm(7, 0, 0);
            setPwm(8, 0, 0);
            if (col == COLOR.red) {
                setPwm(7, 0, L_brightness);
            }
            if (col == COLOR.green) {
                setPwm(6, 0, L_brightness);
            }
            if (col == COLOR.blue) {
                setPwm(8, 0, L_brightness);
            }
            if (col == COLOR.white) {
                setPwm(6, 0, L_brightness);
                setPwm(7, 0, L_brightness);
                setPwm(8, 0, L_brightness);
            }
            if (col == COLOR.black) {
            }
        }
    }
    /**
     * set the rgb-led color via data
     */
    //% block=" set RGBled $RgbLed R:$red G:$green B:$blue"
    //% red.min=0 red.max=255 green.min=0 green.max=255 blue.min=0 blue.max=255
    //% group="RGB-led" weight=77
    export function SetLed(RgbLed: LR, red: number, green: number, blue: number) {
        if (!PCA9685_Initialized) {
            init_PCA9685();
        }

        let R = Math.map(red, 0, 255, 0, L_brightness);
        let G = Math.map(green, 0, 255, 0, L_brightness);
        let B = Math.map(blue, 0, 255, 0, L_brightness);

        if (RgbLed == 0) {    //left side RGB_LED
            setPwm(9, 0, R);
            setPwm(10, 0, G);
            setPwm(11, 0, B);
        }
        if (RgbLed == 1) {    //right side RGB_LED
            setPwm(6, 0, G);
            setPwm(7, 0, R);
            setPwm(8, 0, B);
        }
    }
    /**
     * turn off all rgb-led
     */
    //% block="turn off all RGB-led"
    //% group="RGB-led" weight=76
    export function OFFLed() {
        if (!PCA9685_Initialized) {
            init_PCA9685();
        }
        let led_pin;
        for (led_pin = 6; led_pin <= 11; led_pin++) {
            setPwm(led_pin, 0, 0);
        }
    }

    /////////////////////////////////////////////////////
    //% block="LineTracking"
    //% group="Sensor" weight=69
    export function LineTracking(): number {
        let val = 0;
        /*switch(lt){
            case LT.Left  :
                val = pins.digitalReadPin(DigitalPin.P14);
                break;
            case LT.Center:
                val = pins.digitalReadPin(DigitalPin.P15);
                break;
            case LT.Right :
                val = pins.digitalReadPin(DigitalPin.P16);
                break;
        }*/
        val = (pins.digitalReadPin(DigitalPin.P14)<<2) + 
              (pins.digitalReadPin(DigitalPin.P15)<<1) +
              (pins.digitalReadPin(DigitalPin.P16));
        return val;
    }
    /**
     * Ultrasonic sensor
     */
    let lastTime = 0;
    //% block="Ultrasonic"
    //% group="Sensor" weight=68
    export function ultra(): number {
        //send trig pulse
        pins.setPull(DigitalPin.P1, PinPullMode.PullNone);
        pins.digitalWritePin(DigitalPin.P1, 0)
        control.waitMicros(2);
        pins.digitalWritePin(DigitalPin.P1, 1)
        control.waitMicros(10);
        pins.digitalWritePin(DigitalPin.P1, 0)

        // read echo pulse  max distance : 6m(35000us)  
        let t = pins.pulseIn(DigitalPin.P2, PulseValue.High, 35000);
        let ret = t;

        //Eliminate the occasional bad data
        if (ret == 0 && lastTime != 0) {
            ret = lastTime;
        }
        lastTime = t;

        return Math.round(ret / 58);
    }
    /**
     * A button on the driver board
     */
    //% block="Button"
    //% group="Sensor" weight=66
    export function button(): number {
        return pins.digitalReadPin(DigitalPin.P5);
    }

}

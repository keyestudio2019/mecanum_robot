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
    // If you wanted to write some code that stepped through the servos then this is the BASe and size to do that   
    let Servo1RegBase = 0x08
    let ServoRegDistance = 4
    //To get the PWM pulses to the correct size and zero offset these are the default numbers. 
    let ServoMultiplier = 226
    let ServoZeroOffset = 0x66
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
    let initalised = false //a flag to allow us to initialise without explicitly calling the secret incantation
    //nice big list of servos for the block to use. These represent register offsets in the PCA9865
    export enum Servos {
        // Servo5 = 0x1C,        
        // Servo6 = 0x20,        
        // Servo7 = 0x24,
        // Servo8 = 0x28,
        // Servo9 = 0x2C,
        // Servo10 = 0x30,
        // Servo11 = 0x34,
        // Servo12 = 0x38,
        // Servo13 = 0x3C,
        Servo14 = 0x40,
        Servo15 = 0x44,
    }

    export enum BoardAddresses {
        Board1 = 0x47,
    }
    //Trim the servo pulses. These are here for advanced users, and not exposed to blocks.
    //It appears that servos I've tested are actually expecting 0.5 - 2.5mS pulses, 
    //not the widely reported 1-2mS 
    //that equates to multiplier of 226, and offset of 0x66
    // a better trim function that does the maths for the end user could be exposed, the basics are here 
    // for reference

    export function TrimServoMultiplier(Value: number) {
        if (Value < 113) {
            ServoMultiplier = 113
        }
        else {
            if (Value > 226) {
                ServoMultiplier = 226
            }
            else {
                ServoMultiplier = Value
            }

        }
    }
    export function TrimServoZeroOffset(Value: number) {
        if (Value < 0x66) {
            ServoZeroOffset = 0x66
        }
        else {
            if (Value > 0xCC) {
                ServoZeroOffset = 0xCC
            }
            else {
                ServoZeroOffset = Value
            }

        }
    }

    /*
        This secret incantation sets up the PCA9865 I2C driver chip to be running at 50Hx pulse repetition, and then sets the 16 output registers to 1.5mS - centre travel.
        It should not need to be called directly be a user - the first servo write will call it.
    
    */
    function secretIncantation(): void {
        let buf = pins.createBuffer(2)

        //Should probably do a soft reset of the I2C chip here when I figure out how

        // First set the prescaler to 50 hz
        buf[0] = PRESCALE
        buf[1] = 0x85
        pins.i2cWriteBuffer(PCA9685_ADDRESS, buf, false)
        //Block write via the all leds register to set all of them to 90 degrees
        buf[0] = 0xFA
        buf[1] = 0x00
        pins.i2cWriteBuffer(PCA9685_ADDRESS, buf, false)
        buf[0] = 0xFB
        buf[1] = 0x00
        pins.i2cWriteBuffer(PCA9685_ADDRESS, buf, false)
        buf[0] = 0xFC
        buf[1] = 0x66
        pins.i2cWriteBuffer(PCA9685_ADDRESS, buf, false)
        buf[0] = 0xFD
        buf[1] = 0x00
        pins.i2cWriteBuffer(PCA9685_ADDRESS, buf, false)
        //Set the mode 1 register to come out of sleep
        buf[0] = MODE1
        buf[1] = 0x81
        pins.i2cWriteBuffer(PCA9685_ADDRESS, buf, false)
        //set the initalised flag so we dont come in here again automatically
        initalised = true
    }


    /**
         * sets the requested servo to the reguested angle.
         * if the PCA has not yet been initialised calls the initialisation routine
         *
         * @param Servo Which servo to set
         * @param position the angle to set the servo to
         */
    //% blockId=BEP_I2Cservo_write
    //% block="Put %Servo|on position %position"
    //% group="Servo motors"
    //% position.min=0 position.max=2

    export function servoWrite(Servo: Servos, position: number): void {
        if (initalised == false) {
            secretIncantation()
        }

        // Calculate the right degrees, based on 4 step input: 1 = 15, 2 = 52,  3 = 89, 4 = 126, 5 = 163 with base = 15 degrees, intermdiate steps = 37
        let choice = position
        let degrees2 = position

        if (choice <= 0) {
            degrees2 = 50
        }
        else if (choice == 1) {
            degrees2 = 90
        }
        else if (choice >= 2) {
            degrees2 = 125
        }
        //else if(choice >=3){
        //degrees2 = 110
        //}
        //else if(choice == 4){
        //degrees2 = 135
        //}


        let buf = pins.createBuffer(2)
        let HighByte = false
        let deg100 = degrees2 * 100
        let PWMVal100 = deg100 * ServoMultiplier
        let PWMVal = PWMVal100 / 10000
        PWMVal = Math.floor(PWMVal)
        PWMVal = PWMVal + ServoZeroOffset
        if (PWMVal > 0xFF) {
            HighByte = true
        }
        buf[0] = Servo
        buf[1] = PWMVal
        pins.i2cWriteBuffer(PCA9685_ADDRESS, buf, false)
        if (HighByte) {
            buf[0] = Servo + 1
            buf[1] = 0x01
        }
        else {
            buf[0] = Servo + 1
            buf[1] = 0x00
        }
        pins.i2cWriteBuffer(PCA9685_ADDRESS, buf, false)
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


//% color="#AA278D"

let _lTicks: number = 0
let _rTicks: number = 0
let _lTurns: number = 0.0
let _rTurns: number = 0.0
let _lerrTicks: number = 0;
let _rerrTicks: number = 0;
let _lenc: DigitalPin
let _renc: DigitalPin
let _partialTurn: number = 0.0
let _kp: number = 5.5;

enum motorChoice {
    //% block="left"
    Left = 8448,
    //% block="right"
    Right = 8192,
    //% block="both"
    Both = 5555
}

enum motorDir {
    //% block="Forward"
    fwd,
    //% block="Backward"
    bak
}

enum encPin {
    //% block="P0"
    P0 = DigitalPin.P0,
    //% block=P1"
    P1 = DigitalPin.P1,
    //% block="P2"
    P2 = DigitalPin.P2,
    //% block="P8"
    P8 = DigitalPin.P8,
    //% block="P12"
    P12 = DigitalPin.P12,
    //% blcok="P14"
    P14 = DigitalPin.P14
}

enum MotorPower {
    //%block="ON"
    On = 28673,
    //%block="OFF"
    Off = 28672
}

control.onEvent(EventBusSource.MICROBIT_ID_IO_P0, EventBusValue.MICROBIT_PIN_EVT_RISE, function () {
    _lTicks += 1;
    _lerrTicks += 1;
    if (_lTicks % _partialTurn == 0) {
        _lTicks = 0;
        _lTurns += .0625;
    }
})

control.onEvent(EventBusSource.MICROBIT_ID_IO_P1, EventBusValue.MICROBIT_PIN_EVT_RISE, function () {
    _rTicks += 1;
    _rerrTicks += 1;
    if (_rTicks % _partialTurn == 0) {
        _rTicks = 0;
        _rTurns += .0625;
    }
})





namespace encMotor {

    //%
    export class Robot { }
    let _ratio: number;
    let _lenc: DigitalPin;
    let _renc: DigitalPin;
    let _baseSp: number;

    /**
     * Creates a robot and automtically set it to a variable
     * @param ratio gives the motor to wheel turn ratio eg:48
     */
    //% block="create robot with %ratio to 1 gearing"
    //% blockSetVariable=robot
    export function createRobot(ratio: number): Robot {
        _ratio = ratio;
        _lenc = DigitalPin.P0;
        _renc = DigitalPin.P1;
        pins.setPull(_lenc, PinPullMode.PullUp)
        pins.setEvents(_lenc, PinEventType.Edge)
        pins.setPull(_renc, PinPullMode.PullUp)
        pins.setEvents(_renc, PinEventType.Edge)
        _partialTurn = (_ratio * 4) / 16
        return undefined;

    }

    /**
     * Moves a robot based on wheel rotations
     * @param rt indicates number of rotations eg:4
     */
    //% block="move %robot=variables_get(robot) %motor %dir for %rt Rotations"
    export function drive(robot: Robot, motor: motorChoice, dir: motorDir, rt: number) {
        _lTurns = 0;
        _rTurns = 0;
        _lTicks = 0;
        _rTicks = 0;
        _lerrTicks = 0;
        _rerrTicks = 0;
        _baseSp = 35;
        let lSpeed = _baseSp;
        let correction = 0.0;
        if (motor == motorChoice.Both) {

            for (let i = 0; i <= _baseSp; i += 5) {
                _rerrTicks = 0;
                _lerrTicks = 0;
                correction = (_rerrTicks - _lerrTicks) / _kp;
                motorGo(i, 8192, dir) //start right motor
                motorGo(i += correction, 8448, dir) //start left motor
                basic.pause(80);
            }

        }
        else {
            motorGo(50, motor, dir)
        }


        while ((_lTurns < (rt + .05)) && (_rTurns < (rt + .05))) {
            _rerrTicks = 0;
            _lerrTicks = 0;
            basic.pause(125)
            if (motor == motorChoice.Both) {
                correction = (_rerrTicks - _lerrTicks) / _kp;
                lSpeed += correction;
                motorGo(lSpeed, 8448, dir) //correct left motor
            }

        }
        stop()



    }

    //% block="drive %motorChoice motor(s) %motorDir for %tm secs."
    //% tm.defl=5
    export function driveWtime(motor: motorChoice, dir: motorDir, tm: number) {
        _lTurns = 0;
        _rTurns = 0;
        _lTicks = 0;
        _rTicks = 0;
        if (motor == motorChoice.Both) {
            motorGo(50, 8448, dir) //start left motor
            motorGo(50, 8192, dir) //start right motor
        }
        else { motorGo(50, motor, dir) }
        basic.pause(tm * 1000)
        stop()
        _lTurns = 0;
        _rTurns = 0;
        _lTicks = 0;
        _rTicks = 0;
    }

    //% block="drive %motorChoice motor(s) %motorDir with pwr %power"
    //% power.min=0 power.max=100 power.defl=50
    export function driveIndef(motor: motorChoice, dir: motorDir, power: number) {
        stop();
        _lTurns = 0;
        _rTurns = 0;
        _lTicks = 0;
        _rTicks = 0;
        if (motor == motorChoice.Both) {
            motorGo(power, 8448, dir) //start left motor
            motorGo(power, 8192, dir) //start right motor
        }
        else { motorGo(power, motor, dir) }
    }

    //% block="stop motors"
    export function stop() {
        pins.i2cWriteNumber(89, MotorPower.Off, NumberFormat.Int16BE)//stop motors
        motorGo(0, 8448, 0) //set left speed to 0
        motorGo(0, 8192, 0) //set right speed to 0
    }

    function motorGo(sp: number, mt: number, dir: number) {
        pins.i2cWriteNumber(89, 28673, NumberFormat.Int16BE) //enable motors
        pins.i2cWriteNumber(89, (mt + pwr(dir, sp)), NumberFormat.Int16BE) //start designated motor
    }





    function pwr(dir: number, speed: number): number {
        let outPwr: number = 0

        speed = Math.abs(speed)
        if (speed > 100) {
            speed = 100
        }

        if (dir == motorDir.fwd) {
            outPwr = pins.map(speed, 0, 100, 0, 127)
            outPwr = 128 + outPwr
        }
        else {
            outPwr = pins.map(speed, 0, 100, 127, 0)
        }

        return outPwr
    }
}  
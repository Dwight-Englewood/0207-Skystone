package org.firstinspires.ftc.teamcode.Autonomous.Methods;

import com.qualcomm.hardware.bosch.BNO055IMU;

import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Hardware.Movement;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.util.Range;

public class NewAutonMethods {
    public DcMotor
            BL, BR, FL, FR, lift, intakeL, intakeR, tape;

    public static Servo
            closer, hinger, foundationLeft, foundationRight, leftPurp, rightPurp, leftBlue, rightBlue, spinner, grabber;

    HardwareMap map;
    Telemetry tele;

    public ElapsedTime runtime = new ElapsedTime();

    public int command, target, current;
    private int originTick;
    public int origin;
    float curHeading;

    double power;
    public double kpVal = 0.00005, kiVal = 0.00000045*4, kdVal = 0.00004;
    public double lastError, error = 0, errorI = 0, errorD = 0;

    public final double TkpVal = 0.02, TkiVal = 0.000018, TkdVal = 0.0006;
    public double TlastError, Terror = 0, TerrorI = 0, TerrorD = 0;

    final double theoryVal = 0.0003;
    final double lowerBound = 0.235;
    final double upperBound = 0.85;

  //  public RevBlinkinLedDriver blinkin;
    public static BNO055IMU gyro;

    public boolean strafe = false, backLeft = false;

    public NewAutonMethods() {
        command = 0;
    }

    /**
     * inits hardware
     *
     * @param map  creates object on phones config
     */
    public void init(HardwareMap map, Telemetry tele) {
        this.map = map;
        this.tele = tele;

        BR = this.map.get(DcMotor.class, "BR");
        BL = this.map.get(DcMotor.class, "BL");
        FL = this.map.get(DcMotor.class, "FL");
        FR = this.map.get(DcMotor.class, "FR");

        lift = this.map.get(DcMotor.class, "Lift");
        intakeL = this.map.get(DcMotor.class, "intakeL");
        intakeR = this.map.get(DcMotor.class, "intakeR");

        hinger = this.map.get(Servo.class, "hinger");
        foundationLeft = this.map.get(Servo.class, "fleft");
        foundationRight = this.map.get(Servo.class, "fright");

        closer = this.map.get(Servo.class, "closer");

        lift.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeL.setDirection((DcMotorSimple.Direction.FORWARD));
        intakeR.setDirection((DcMotorSimple.Direction.REVERSE));

        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.FORWARD);
        FL.setDirection(DcMotorSimple.Direction.FORWARD);
        FR.setDirection(DcMotorSimple.Direction.REVERSE);

        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.changeRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.openServoAuton();
    }

    public void initNew(HardwareMap map, Telemetry tele) {
        this.map = map;

        BR = this.map.get(DcMotor.class, "BR");
        BL = this.map.get(DcMotor.class, "BL");
        FL = this.map.get(DcMotor.class, "FL");
        FR = this.map.get(DcMotor.class, "FR");

        lift = this.map.get(DcMotor.class, "lift");
        intakeL = this.map.get(DcMotor.class, "intakeL");
        intakeR = this.map.get(DcMotor.class, "intakeR");

        BR.setDirection(DcMotorSimple.Direction.FORWARD);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        FR.setDirection(DcMotorSimple.Direction.FORWARD);

        lift.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeL.setDirection((DcMotorSimple.Direction.REVERSE));
        intakeR.setDirection((DcMotorSimple.Direction.REVERSE));

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        foundationLeft = this.map.get(Servo.class, "fleft");
        foundationRight = this.map.get(Servo.class, "fright");

        leftBlue = this.map.get(Servo.class, "leftBlue");
        leftPurp = this.map.get(Servo.class, "leftPurp");
        rightBlue = this.map.get(Servo.class, "rightBlue");
        rightPurp = this.map.get(Servo.class, "rightPurp");

        hinger = this.map.get(Servo.class, "hinge");
        spinner = this.map.get(Servo.class, "spinner");
        grabber = this.map.get(Servo.class, "grabber");

        this.changeRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.newOpenServoAuton();
    }

    public void initGyro(){
        gyro = map.get(BNO055IMU.class, "gyro");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        gyro.initialize(parameters);
        curHeading = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    public boolean isGyroInit(){
        return gyro.isGyroCalibrated();
    }

    /**
     * Changes the encoder state of all four motors.
     *
     * @param runMode a dc motor run mode.
     */
    public void changeRunMode(DcMotor.RunMode runMode) {
        BL.setMode(runMode);
        BR.setMode(runMode);
        FL.setMode(runMode);
        FR.setMode(runMode);
    }

    /**
     * @param in powerlevel
     */
    public void drive(double in) {
        BL.setPower(in);
        BR.setPower(in);
        FR.setPower(in);
        FL.setPower(in);
    }

    public void strafeDrive(double in) { //RightStrafe
        BL.setPower(-in);
        BR.setPower(in);
        FR.setPower(-in);
        FL.setPower(in);
    }

    public void autonDrive(Movement movement, int target) {
        switch (movement) {
            case FORWARD:
                FL.setTargetPosition(target);
                FR.setTargetPosition(target);
                BL.setTargetPosition(target);
                BR.setTargetPosition(target);
                break;

            case BACKWARD:
                FL.setTargetPosition(-target);
                FR.setTargetPosition(-target);
                BL.setTargetPosition(-target);
                BR.setTargetPosition(-target);
                break;

            case LEFTSTRAFE:
                FL.setTargetPosition(-target);
                FR.setTargetPosition(target);
                BL.setTargetPosition(target);
                BR.setTargetPosition(-target);
                strafe = true;
                break;

            case RIGHTSTRAFE:
                FL.setTargetPosition(target);
                FR.setTargetPosition(-target);
                BL.setTargetPosition(-target);
                BR.setTargetPosition(target);
                strafe = true;
                break;

            case UPRIGHT:
                FL.setTargetPosition(target);
                BR.setTargetPosition(target);

                FR.setTargetPosition(FR.getCurrentPosition());
                BL.setTargetPosition(BL.getCurrentPosition());
                backLeft = false;
                break;

            case UPLEFT:
                FR.setTargetPosition(target);
                BL.setTargetPosition(target);

                FL.setTargetPosition(FL.getCurrentPosition());
                BR.setTargetPosition(BR.getCurrentPosition());
                backLeft = true;
                break;

            case DOWNRIGHT:
                FR.setTargetPosition(-target);
                BL.setTargetPosition(-target);

                FL.setTargetPosition(FL.getCurrentPosition());
                BR.setTargetPosition(BR.getCurrentPosition());
                backLeft = true;
                break;

            case DOWNLEFT:
                FL.setTargetPosition(-target);
                BR.setTargetPosition(-target);

                FR.setTargetPosition(FR.getCurrentPosition());
                BL.setTargetPosition(BL.getCurrentPosition());
                backLeft = false;
                break;

            case LEFTTURN:
                FL.setTargetPosition(-target);
                FR.setTargetPosition(-target);
                BL.setTargetPosition(target);
                BR.setTargetPosition(target);
                break;

            case RIGHTTURN:
                FL.setTargetPosition(target);
                FR.setTargetPosition(target);
                BL.setTargetPosition(-target);
                BR.setTargetPosition(-target);
                break;

            case STOP:
                FL.setTargetPosition(FL.getCurrentPosition());
                FR.setTargetPosition(FR.getCurrentPosition());
                BL.setTargetPosition(BL.getCurrentPosition());
                BR.setTargetPosition(BR.getCurrentPosition());
                break;
        }
    }

    public void drive(Movement movement, double power) {
        switch (movement) {
            case FORWARD:
                FL.setPower(power);
                FR.setPower(power);
                BL.setPower(power);
                BR.setPower(power);
                break;

            case BACKWARD:
                FR.setPower(power);
                FL.setPower(power);
                BL.setPower(-power);
                BR.setPower(-power);
                break;

            case LEFTSTRAFE:
                FL.setPower(power);
                FR.setPower(-power);
                BL.setPower(-power);
                BR.setPower(power);
                break;

            case RIGHTSTRAFE:
                FL.setPower(-power);
                FR.setPower(power);
                BL.setPower(power);
                BR.setPower(-power);
                break;

            case UPRIGHT:
                FL.setPower(power);
                BR.setPower(power);

                FR.setPower(0);
                BL.setPower(0);
                break;

            case UPLEFT:
                FR.setPower(power);
                BL.setPower(power);

                FL.setPower(0);
                BR.setPower(0);
                break;

            case DOWNRIGHT:
                FR.setPower(-power);
                BL.setPower(-power);

                FL.setPower(0);
                BR.setPower(0);
                break;

            case DOWNLEFT:
                FL.setPower(-power);
                BR.setPower(-power);

                FR.setPower(0);
                BL.setPower(0);
                break;

            case LEFTTURN:
                FL.setPower(power);
                FR.setPower(-power);
                BL.setPower(power);
                BR.setPower(-power);
                break;

            case RIGHTTURN:
                FL.setPower(-power);
                FR.setPower(power);
                BL.setPower(-power);
                BR.setPower(power);
                break;

            case STOP:
                FL.setPower(0);
                FR.setPower(0);
                BL.setPower(0);
                BR.setPower(0);
                break;
        }
    }

    public boolean adjustHeading(int targetHeading) {
        double headingError;
        curHeading = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        headingError = targetHeading - curHeading;
        this.changeRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (headingError < -1 && headingError > -180)
            this.drive(Movement.LEFTTURN, scaleTurn(targetHeading));
        else if (headingError > 1 && headingError < 180) {
            this.drive(Movement.RIGHTTURN, scaleTurn(targetHeading));
        } else {
            return true;
        }
        return false;
    }
    //Positive is left turn, negative is right turn.

    public double strafeVal(double target) {
        return (target * 1.2);
    }

    public void encoderReset() {
        this.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.runtime.reset();
        this.command++;
    }

    public void liftReset() {
        this.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.command++;
    }

    public void intakeReset() {
        this.intakeL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.intakeR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.command++;
    }

    public void runtimeReset(){ this.runtime.reset();
    }

    public void closeHingeAuton() {
        this.hinger.setPosition(0);
    }

    public void openHingeAuton() {
        this.hinger.setPosition(0.75);
    }

    public void openClampAuton() {
        this.closer.setPosition(1);
    }

    public void closeClampAuton() {
        this.closer.setPosition(0);
    }

    public void openServoAuton() {
        this.foundationLeft.setPosition(0);
        this.foundationRight.setPosition(1);
    }

    public void closeServoAuton() {
        this.foundationLeft.setPosition(0.57); //0.55 //right
        this.foundationRight.setPosition(0.32); //0.35 //left
    }

    public void newCloseServoAuton() {
        this.foundationLeft.setPosition(1);
        this.foundationRight.setPosition(0);
    }

    public void newOpenServoAuton() {
        this.foundationLeft.setPosition(0);
        this.foundationRight.setPosition(1);
    }

    public void liftRightClaws(){
        this.leftBlue.setPosition(0);
        this.leftPurp.setPosition(0.5);
   //     this.rightBlue.setPosition(1);
   //     this.rightPurp.setPosition(0.5);
    }

    public void lowerRightClaws(){
        this.leftBlue.setPosition(1);
        this.leftPurp.setPosition(0.08);
 //       this.rightBlue.setPosition(0);
 //       this.rightPurp.setPosition(1);
    }

    public void closeRightClaws(){
        this.leftBlue.setPosition(0);
  //      this.rightBlue.setPosition(1);
    }

    public void openRightClaws(){
        this.leftBlue.setPosition(1);
        //      this.rightBlue.setPosition(1);
    }

    public void lowerClosedClaws(){
        this.leftPurp.setPosition(0.08);
    }

    public void lowerLift(double height) {
        this.lift.setDirection(DcMotorSimple.Direction.FORWARD);
        this.lift.setTargetPosition(cmDistance(height));
        this.lift.setPower(-0.8);
        this.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void raiseLiftDeux(double height) {
        this.lift.setDirection(DcMotorSimple.Direction.FORWARD);
        this.lift.setTargetPosition(cmDistance(height));
        this.lift.setPower(0.5);
        this.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void lowerLiftDeux(double height) {
        this.lift.setDirection(DcMotorSimple.Direction.REVERSE);
        this.lift.setTargetPosition(cmDistance(height));
        this.lift.setPower(0.5);
        this.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /*public void percentagePower() {
        int target = FL.getTargetPosition();
        int current = FL.getCurrentPosition();
        this.error = Math.abs(cmDistance(target - current)); //Distance from current position to end position
        int pointToOriginFL = Math.abs(cmDistance(this.origin - current));  //Distance from current position to start position
        double totalDistanceFL = Math.abs(cmDistance(this.error + pointToOriginFL));
        runtimeReset();

        if (pointToOriginFL < error) { //Startpoint to Midpoint
            if (pointToOriginFL == 0) { //Startpoint
                power = 0.1;
            } else if (this.error - pointToOriginFL < 0.5 * totalDistanceFL) { //Quarter to Midpoint
                power = (this.error * theoryVal);
                if (Math.abs(this.error - pointToOriginFL) < 0.2 * totalDistanceFL) { // 0.4 to 0.5 point
                    power = 1;
                }
            } else if (this.error - pointToOriginFL > 0.2 * totalDistanceFL) { // Startpoint to 0.4
                power = (pointToOriginFL * theoryVal);
                if (power < lowerBound) {
                    power = lowerBound;
                } else {
                    power = (pointToOriginFL * theoryVal);
                }
            }
        } else if (pointToOriginFL > this.error) { //Midpoint to Endpoint
            if (pointToOriginFL - this.error < 0.5 * totalDistanceFL) { //Midpoint to Three Quarter
                power = (this.error * theoryVal);
                if (Math.abs(this.error - pointToOriginFL) < 0.2 * totalDistanceFL) { // 0.5 to 0.6 point
                    power = 1;
                }
            } else if (Math.abs(this.error - pointToOriginFL) > 0.2 * totalDistanceFL) { //0.6 to final
                power = (this.error * theoryVal);
                if (power > lowerBound) {
                    power = lowerBound;
                } else {
                    power = (pointToOriginFL * theoryVal);
                    if (power > lowerBound) {
                        power = lowerBound;
                    }
                }
            } else {
                power = (this.error * theoryVal);
            }
        } else {
            power = upperBound;
        }

        tele.addData("error", this.error);
        tele.addData("pointToOrigin", pointToOriginFL);
        tele.addData("power", power);
        if (power == .25){
            if (runtime.milliseconds() >= 1000){
                this.drive(power + .1);
            } else {
                this.drive(power);
            }
        } else {
            this.drive(power);
        }
    }

     */

    /*public void robinPower() {
        double power;
        int tartar = FL.getTargetPosition();
        int curcur = FL.getCurrentPosition();
        int diff = Math.abs(tartar - curcur);
        int originDiff = Math.abs(this.originTick - curcur);

        if (originDiff < 75) {
            power = .1;
        } else if (originDiff < 250) {
            power = .3;
        } else if (originDiff < 400) {
            power = .5;
        } else {
            power = 1;
        }

        if (diff < 100) {
            power = .1;
        } else if (diff < 300) {
            power = .3;
        } else if (diff < 500) {
            power = .5;
        } else if (diff < 825) {
            power = .7;
        }

        this.drive(power);
    }

     */

    public double scalePower() {
        if (!backLeft) {
            target = cmDistance(BR.getTargetPosition());
            current = cmDistance(BR.getCurrentPosition());
        } else {
            target = cmDistance(BL.getTargetPosition());
            current = cmDistance(BL.getCurrentPosition());
        }

        if (Math.abs(this.BL.getTargetPosition() - this.BL.getCurrentPosition()) <= 25){
            kpVal *= 0.3;
        } else if (Math.abs(this.BL.getTargetPosition() - this.BL.getCurrentPosition()) <= 100){
            kpVal *= 0.45;
        } else if (Math.abs(this.BL.getTargetPosition() - this.BL.getCurrentPosition()) <= 200){
            kpVal *= 0.6;
        } else {
            kpVal = 0.00005;
        }

        if ((this.error * this.kpVal) + (this.errorI * this.kiVal) - (this.errorD * this.kdVal) >= .8) {
            this.errorI += 0;
        } else {
            this.errorI = this.errorI + this.error;
        }

        this.error = (target - current); //Distance from current position to end position
        this.errorD = (current - this.lastError);
        this.lastError = current;

        return Range.clip((this.error * this.kpVal) + (this.errorI * this.kiVal) - (this.errorD * this.kdVal), -1, 1);
    }

    public double scaleTurn(double targHeading){
        curHeading = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        if ((this.Terror * this.TkpVal) + (this.TerrorI * this.TkiVal) - (this.TerrorD * this.TkdVal) > 1) {
            this.TerrorI += 0;
        } else {
            this.TerrorI = this.TerrorI + this.Terror;
        }

        this.Terror = (targHeading - curHeading); //Distance from current position to end position
        this.TerrorD = (curHeading - this.TlastError);
        this.TlastError = curHeading;

        return Math.abs(Range.clip((this.Terror * this.TkpVal) + (this.TerrorI * this.TkiVal) - (this.TerrorD * this.TkdVal), -1, 1));
    }

    public void finishDrive(){
        if (backLeft) {
            if (Math.abs(this.BL.getTargetPosition() - this.BL.getCurrentPosition()) <= 20
                    && Math.abs(this.BR.getTargetPosition() - this.BR.getCurrentPosition()) <= 20
                    && Math.abs(this.FL.getTargetPosition() - this.FL.getCurrentPosition()) <= 20
                    && Math.abs(this.FR.getTargetPosition() - this.FR.getCurrentPosition()) <= 20) {
                this.drive(Movement.STOP, 0);
                this.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                this.lastError = this.error;
                this.errorI = 0;
                this.error = 0;
                this.runtime.reset();
                this.command++;
            } else {
                this.drive(scalePower());
            }
        } else {
            if (Math.abs(this.BL.getTargetPosition() - this.BL.getCurrentPosition()) <= 20
                    && Math.abs(this.BR.getTargetPosition() - this.BR.getCurrentPosition()) <= 20
                    && Math.abs(this.FL.getTargetPosition() - this.FL.getCurrentPosition()) <= 20
                    && Math.abs(this.FR.getTargetPosition() - this.FR.getCurrentPosition()) <= 20) {

                this.drive(Movement.STOP, 0);
                this.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                this.lastError = this.error;
                this.errorI = 0;
                this.error = 0;
                this.runtime.reset();
                this.command++;
            } else {
                this.drive(scalePower());
            }
        }
    }

    public void gyroTurn(int turn) {
        if (adjustHeading(turn)) {
            this.drive(Movement.STOP, 0);
            this.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            this.TlastError = this.Terror;
            this.TerrorI = 0;
            this.Terror = 0;
            this.runtime.reset();
            this.command++;
        } else {
            adjustHeading(turn);
        }
    }

    public void setTarget(Movement movement, int target) {
        this.autonDrive(movement, cmDistance(target));
        this.changeRunMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.originTick = BL.getCurrentPosition();

        if (BL.getTargetPosition() == 0 && BL.getCurrentPosition() == 0) {
            this.originTick = cmDistance(BR.getTargetPosition());
        }
        this.command++;
    }

    public void tapeExtend(int target, double power) {
        this.tape.setTargetPosition(target);
        this.tape.setPower(power);
        this.tape.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if ((Math.abs(this.tape.getCurrentPosition() - this.tape.getTargetPosition()) < 25)) {
            this.tape.setPower(0);
            if (this.tape.getCurrentPosition() != 0) {
                this.tape.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                tele.addLine("Reset Not Successful");
                tele.update();
            } else {
                this.command++;
            }
        }
    }

    private int cmDistance(double distance) {
        final double wheelCirc = 31.9185813;
        final double gearMotorTick = 537.6; //neverrest orbital 20 = 537.6 counts per revolution
        //1:1 gear ratio so no need for multiplier
        return (int) (gearMotorTick * (distance / wheelCirc));
        //rate = x(0.05937236104)
    }
}






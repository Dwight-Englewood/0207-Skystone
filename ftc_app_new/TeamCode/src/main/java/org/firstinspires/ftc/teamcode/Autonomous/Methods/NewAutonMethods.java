package org.firstinspires.ftc.teamcode.Autonomous.Methods;

import com.qualcomm.hardware.bosch.BNO055IMU;

import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Autonomous.Methods.SkystoneDetect;
import org.firstinspires.ftc.teamcode.Hardware.Movement;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.util.Range;

public class NewAutonMethods {
    public static DcMotor
            BL, BR, FL, FR, lift, intakeL, intakeR;

    public static Servo
            closer, hinger, foundationLeft, grabby, flippy, foundationRight;

    HardwareMap map;
    Telemetry tele;

    public ElapsedTime runtime = new ElapsedTime();

    public int command;
    private int originTick;
    int curVal = 0;
    double pVal = 0.0000045;
    double worldXPosition, worldYPosition, worldAngle_rad;

    public RevBlinkinLedDriver blinkin;
    public static BNO055IMU gyro;

    public double oldX = 0;
    public double oldY = 0;
    public double oldAng = 0;
    public double deltaRight, deltaLeft, newX, newY, newTheta, wheelLength;
    public boolean strafe;

    public NewAutonMethods() {
        command = 0;
        worldXPosition = 50;
        worldYPosition = 70;
        worldAngle_rad = Math.toRadians(-180);
    }

    /**
     * inits hardware
     *
     * @param map  creates object on phones config
     * @param tele displays data on phone
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

        blinkin = this.map.get(RevBlinkinLedDriver.class, "rgbReady");

        hinger = this.map.get(Servo.class, "hinger");
        foundationLeft = this.map.get(Servo.class, "fleft");
        foundationRight = this.map.get(Servo.class, "fright");
        flippy = this.map.get(Servo.class, "flippy");
        grabby = this.map.get(Servo.class, "grabby");

        closer = this.map.get(Servo.class, "closer");

        lift.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeL.setDirection((DcMotorSimple.Direction.FORWARD));
        intakeR.setDirection((DcMotorSimple.Direction.REVERSE));

        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.FORWARD);
        FL.setDirection(DcMotorSimple.Direction.FORWARD);
        FR.setDirection(DcMotorSimple.Direction.REVERSE);

        this.changeRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        gyro = this.map.get(BNO055IMU.class, "gyro");
        gyro.initialize(parameters);
        tele.addData(">", "Gyro Calibrating. Do Not Move! (Real Test)");
        tele.update();
    }

    /**
     * Changes the encoder state of all four motors.
     *
     * @param runMode a dc motor run mode.
     */
    public static void changeRunMode(DcMotor.RunMode runMode) {
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
                break;

            case RIGHTSTRAFE:
                FL.setTargetPosition(target);
                FR.setTargetPosition(-target);
                BL.setTargetPosition(-target);
                BR.setTargetPosition(target);
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
                strafe = true;
                break;

            case RIGHTSTRAFE:
                FL.setPower(-power);
                FR.setPower(power);
                BL.setPower(power);
                BR.setPower(-power);
                strafe = true;
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
        double curHeading = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        double headingError;
        headingError = targetHeading - curHeading;
        double driveScale = headingError;
        this.changeRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (headingError < -0.3)
            driveScale = -.5;
        else if (headingError > 0.3)
            driveScale = .5;
        else {
            driveScale = 0;
            this.drive(Movement.RIGHTTURN, driveScale * 0.65);
            return true;
        }
        this.drive(Movement.RIGHTTURN, driveScale * 0.65);
        return false;
    }
    //Positive is left turn, negative is right turn.

    public double strafeVal(double target) {
        return (target * 1.2);
    }

    public void runToTarget(Movement movementEnum, double target) {
        if (strafe) {
            this.autonDrive(movementEnum, cmDistance(strafeVal(target)));
        } else {
            this.autonDrive(movementEnum, cmDistance(target));
        }
        this.scalePower();
        this.changeRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        if ((Math.abs(FL.getCurrentPosition() - FL.getTargetPosition()) < 5) ||
                (Math.abs(BR.getCurrentPosition() - BR.getTargetPosition()) < 5)) {
            autonDrive(movementEnum.STOP, 0);
            tele.update();
            this.command++;
        }
    }

    public void setTarget(double x, double y, double movementSpeed, double preferredAngle, double turnSpeed) {
        double distance = Math.hypot(x-worldXPosition, y-worldYPosition);
        double absoluteAngleToTarget = Math.atan2(y-worldYPosition, x-worldXPosition);
        double relativeAngleToPoint = this.AngleWrap(absoluteAngleToTarget - (worldAngle_rad - Math.toRadians(90)));
        double relativeXToPoint = Math.cos(relativeAngleToPoint) * distance;
        double relativeYToPoint = Math.sin(relativeAngleToPoint) * distance;

        worldXPosition = relativeXToPoint;
        worldYPosition = relativeYToPoint;

        double movementXPower = relativeXToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
        double movementYPower = relativeYToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));

        double movement_x = movementXPower * movementSpeed;
        double movement_y = movementYPower * movementSpeed;

        double relativeTurnAngle = relativeAngleToPoint - Math.toRadians(180) + preferredAngle;

        double movement_turn = Range.clip(relativeTurnAngle/Math.toRadians(30),-1,1) * turnSpeed;

        this.FL.setPower(movement_y + movement_x - movement_turn);
        this.FR.setPower(movement_y - movement_x + movement_turn);
        this.BL.setPower(movement_y - movement_x - movement_turn);
        this.BR.setPower(movement_y + movement_x + movement_turn);

        if (Math.abs(x-worldXPosition) < 5|| Math.abs(y-worldYPosition) < 5){
            movementSpeed = 0;
            movement_turn = 0;
            this.command++;

        }

        if (distance < 8){
            movement_turn = 0;
            this.command++;
        }

        tele.addData("movement_y", movement_y);
        tele.addData("movement_x", movement_x);
        tele.addData("movement_turn", movement_turn);

        tele.addData("x", x);
        tele.addData("y", y);

        tele.addData("relativeXToPoint", relativeXToPoint);
        tele.addData("relativeYToPoint", relativeYToPoint);

        tele.addData("movement_turn", movement_turn);
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

    public void intakeAuton(int target, double power) {
        this.intakeL.setTargetPosition(cmDistance(target));
        this.intakeR.setTargetPosition(cmDistance(target));

        this.intakeL.setPower(power);
        this.intakeR.setPower(power);

        this.intakeL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.intakeR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if ((Math.abs(this.intakeL.getCurrentPosition() - this.intakeL.getTargetPosition()) < 5) || (Math.abs(this.intakeR.getCurrentPosition() - this.intakeR.getTargetPosition()) < 5)) {
            this.intakeL.setPower(0);
            this.intakeR.setPower(0);
            this.command++;
        }
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

    public void scalePower() {
        double power;
        int target = FL.getTargetPosition();
        int current = FL.getCurrentPosition();
        int diff = cmDistance(Math.abs(target - current)); //Distance from current position to end position
        int originDiff = cmDistance(Math.abs(this.originTick - current));  //Distance from current position to start position

        if (originDiff < diff) { //Startpoint to Midpoint
            if (originDiff == 0) { //Startpoint
                power = 0.05;
            } else if (diff - originDiff < 0.5 * Math.abs(diff + originDiff)) { //Quarter to Midpoint
                power = diff * pVal;
                if (Math.abs(diff - originDiff) < 0.2 * Math.abs(diff + originDiff)) { // 0.4 to 0.6 point
                    power = .8;
                }
                if (power > .8){
                    power = .8;
                } else if (power < .09){
                    power = .09;
                }
            }  else { //Startpoint to Quarter
                power = originDiff * pVal;
                if (power > .8){
                    power = .8;
                }else if (power < .09){
                    power = .09;
                }
            }
        }

        if (originDiff > diff) { //Midpoint to Endpoint
            if (originDiff - diff < 0.5 * Math.abs(diff + originDiff)) { //Midpoint to Three Quarter
                power = diff * pVal;
                if (Math.abs(diff - originDiff) < 0.2 * Math.abs(diff + originDiff)) { // 0.4 to 0.6 point
                    power = .8;
                }
                if (power > .8){
                    power = .8;
                } else if (power < .09){
                    power = .09;
                }
            } else { //Quarter to final
                power = diff * pVal * 30;
                if (power > .8){
                    power = .8;
                } else if (power < .09){
                    power = .09;
                }
            }
        } else { //Midpoint
            power = .8;
        }

        tele.addData("diff", diff);
        tele.addData("originDiff", originDiff);
        tele.addData("power", power);
        this.drive(power);
    }
    /*

        if (originDiff < 75) { //Distance from current position to start position
            power = .7;
        } else if (originDiff < 250) {
            power = .7;
        } else if (originDiff < 400) {
            power = .7;
        } else {
            power = 1;
        }

        if (diff < 50) { //Distance from current position to end position
            power = .02;
        } else if (diff < 100) {
            power = .05;
        } else if (diff < 200) {
            power = .1;
        } else if (diff < 300) {
            power = .3;
        } else if (diff < 500) {
            power = .7;
        } else if (diff < 750) {
            power = .7;
        }
        this.drive(power);
    }

     */

/*
    public void tapeExtend(int target, double power) {
        this.tape.setTargetPosition(target);
        this.tape.setPower(power);
        this.tape.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if ((Math.abs(this.tape.getCurrentPosition() - this.tape.getTargetPosition()) < 25)) {
            this.tape.setPower(0);
            this.command++;
        }
    }
 */

    public void gyroTurn(int turn) {
        if (Math.abs(turn - this.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle) > 10) {
            this.adjustHeading(turn);
        } else if (Math.abs(turn - this.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle) < 10) {
            //   this.curVal = turn;
            this.drive(Movement.STOP, 0);
            this.command++;
        }
    }

    public double getCurval() {
        return this.curVal;
    }

    public double getAngle() {
        return this.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    private int cmDistance(double distance) {
        final double wheelCirc = 31.9185813;
        final double gearMotorTick = 537.6; //neverrest orbital 20 = 537.6 counts per revolution
        //1:1 gear ratio so no need for multiplier
        return (int) (gearMotorTick * (distance / wheelCirc));
        //rate = x(0.05937236104)
    }
//double relativeAngleToPoint = this.AngleWrap(absoluteAngleToTarget - (worldAngle_rad - Math.toRadians(90)));
    public static double AngleWrap(double angle){
        while (angle > -Math.PI){
            angle += 2*Math.PI;
        }

        while (angle < -Math.PI){
            angle -= 2*Math.PI;
        }
        return angle;
    }

}





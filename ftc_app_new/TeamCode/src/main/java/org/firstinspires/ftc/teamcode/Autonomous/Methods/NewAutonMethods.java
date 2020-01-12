package org.firstinspires.ftc.teamcode.Autonomous.Methods;

import com.qualcomm.hardware.bosch.BNO055IMU;

import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Autonomous.Test.FindSkystone;
import org.firstinspires.ftc.teamcode.Hardware.Movement;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

public class NewAutonMethods {
    public static DcMotor
            BL,
            BR,
            FL,
            FR,
            lift,
            intakeL,
            intakeR;
           /* tape */

    public static Servo
            closer,
            hinger,
            foundationLeft,
            grabby,
            flippy,
            foundationRight;

    HardwareMap map;
    Telemetry tele;

    public ElapsedTime runtime = new ElapsedTime();

    public int command;
    private int originTick;
    int curVal = 0;

    //double error = 180 - gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    //Double turnSpeed = 0.5;
    //Integer angle = -45;
    public RevBlinkinLedDriver blinkin;
    public static BNO055IMU gyro;
    BNO055IMU.Parameters parameters;
    Orientation angles;

    public NewAutonMethods() {
        command = 0;
    }

    public void init(HardwareMap map, Telemetry tele, boolean auton) {

        this.map = map;
        this.tele = tele;

        BR = this.map.get(DcMotor.class, "BR");
            BL = this.map.get(DcMotor.class, "BL");
        FL = this.map.get(DcMotor.class, "FL");
        FR = this.map.get(DcMotor.class, "FR");

     //   tape = this.map.get(DcMotor.class, "tape");
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

    public static void changeRunMode(DcMotor.RunMode runMode) {
        BL.setMode(runMode);
        BR.setMode(runMode);
        FL.setMode(runMode);
        FR.setMode(runMode);
    }

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
                break;

            case RIGHTSTRAFE:
                FL.setPower(-power);
                FR.setPower(power);
                BL.setPower(power);
                BR.setPower(-power);
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
            driveScale = -.25;
        else if (headingError > 0.3)
            driveScale = .25;
        else {
            driveScale = 0;
            this.drive(Movement.RIGHTTURN, driveScale);
            return true;
        }
        this.drive(Movement.RIGHTTURN, driveScale);
        return false;
    }
    //Positive is left turn, negative is right turn.

    public double strafeVal(double target) {
        return (target * 1.2);
    }

    public void runToTarget(Movement movementEnum, double target, boolean strafe) {
        if (strafe) {
            this.autonDrive(movementEnum, cmDistance(strafeVal(target)));
        } else {
            this.autonDrive(movementEnum, cmDistance(target));
        }
        this.scalePower();
        this.changeRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        if ((Math.abs(FL.getCurrentPosition() - FL.getTargetPosition()) < 25) ||
                (Math.abs(BR.getCurrentPosition() - BR.getTargetPosition()) < 25)) {
            autonDrive(movementEnum.STOP, 0);
            this.runtime.reset();
            tele.update();
            this.command++;
        }
    }

    public void turn(Movement movementEnum, double target, double power) {
        this.autonDrive(movementEnum, cmDistance(target));
        this.drive(power);
        this.changeRunMode(DcMotor.RunMode.RUN_TO_POSITION);

//        if ((Math.abs(FL.getCurrentPosition() + 25) >= Math.abs(FL.getTargetPosition()) &&
//                Math.abs(FL.getCurrentPosition() - 25) <= Math.abs(FL.getTargetPosition()))) {
        tele.addData("Delta", Math.abs(FL.getCurrentPosition() - FL.getTargetPosition()));
        if ((Math.abs(FL.getCurrentPosition() - FL.getTargetPosition()) < 25) ||
                (Math.abs(BR.getCurrentPosition() - BR.getTargetPosition()) < 25)) {
            autonDrive(movementEnum.STOP, 0);
            tele.update();
            this.command++;
        }
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

    public void skystoneGrab() {
        this.grabby.setPosition(0);
    }

    public void skystoneRelease() {
        this.grabby.setPosition(1);
    }

    public void skystoneFall() {
        this.flippy.setPosition(1);
    }

    public void skystoneRaise() {
        this.flippy.setPosition(0);
    }

    public void openServoAuton() {
        this.foundationLeft.setPosition(0);
        this.foundationRight.setPosition(1);
    }

    public void closeServoAuton() {
        this.foundationLeft.setPosition(0.55);
        this.foundationRight.setPosition(0.35);
    }

    public void intakeAuton(int target, double power) {
        this.intakeL.setTargetPosition(target);
        this.intakeR.setTargetPosition(target);

        this.intakeL.setPower(power);
        this.intakeR.setPower(power);

        this.intakeL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.intakeR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if ((Math.abs(this.intakeL.getCurrentPosition() - this.intakeL.getTargetPosition()) < 25) ||
                (Math.abs(this.intakeR.getCurrentPosition() - this.intakeR.getTargetPosition()) < 25)) {
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
        int target = FL.getTargetPosition();
        int current = FL.getCurrentPosition();
        int diff = Math.abs(target - current);
        int originDiff = Math.abs(this.originTick - current);
        double power;

        if (originDiff < 75) {
            power = .3;
        } else if (originDiff < 250) {
            power = .45;
        } else if (originDiff < 400) {
            power = .55;
        } else {
            power = .65;
        }

        if (diff < 100) {
            power = .3;
        } else if (diff < 300) {
            power = .4;
        } else if (diff < 500) {
            power = .45;
        } else if (diff < 750) {
            power = .55;
        }
        this.drive(power);
    }

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
}





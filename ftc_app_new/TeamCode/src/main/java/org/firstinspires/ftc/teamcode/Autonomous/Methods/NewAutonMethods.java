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

public class NewAutonMethods {
    public static DcMotor
            BL,
            BR,
            FL,
            FR,
            lift,
            intakeL,
            intakeR,
            tape;

    public static Servo
            hinge,
            clamp,
            foundationLeft,
            foundationRight;
    HardwareMap map;
    Telemetry tele;

    public ColorSensor color_sensor;

    public ElapsedTime runtime = new ElapsedTime();
    FindSkystone skystoneFind = new FindSkystone();

    final double proportionalValue = 0.000005;
    public int command;
    private int originTick;
    int curVal = 0;

    //double error = 180 - gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    //Double turnSpeed = 0.5;
    //Integer angle = -45;
    public static BNO055IMU gyro;
    BNO055IMU.Parameters parameters;
    Orientation angles;

    public NewAutonMethods() {
        command = 0;
    }

    public void init(HardwareMap map, Telemetry tele, boolean auton) {
        this.map = map;
        this.tele = tele;

        color_sensor = this.map.get(ColorSensor.class, "Col");

        BR = this.map.get(DcMotor.class, "BR");
        BL = this.map.get(DcMotor.class, "FR");
        FL = this.map.get(DcMotor.class, "FL");
        FR = this.map.get(DcMotor.class, "BL");

        tape = this.map.get(DcMotor.class, "tape");
        lift = this.map.get(DcMotor.class, "Lift");
        intakeL = this.map.get(DcMotor.class, "intakeL");
        intakeR = this.map.get(DcMotor.class, "intakeR");

        clamp = this.map.get(Servo.class, "clamp");
        foundationLeft = this.map.get(Servo.class, "fleft");
        foundationRight = this.map.get(Servo.class, "fright");

        hinge = this.map.get(Servo.class, "hinge");

        lift.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeL.setDirection((DcMotorSimple.Direction.FORWARD));
        intakeR.setDirection((DcMotorSimple.Direction.REVERSE));

        this.changeRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        gyro = this.map.get(BNO055IMU.class, "gyro");
        gyro.initialize(parameters);
        tele.addData(">", "Gyro Calibrating. Do Not Move!");
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

    public double motorSpeed() {
        if (Math.abs(FL.getCurrentPosition()) < Math.abs(FL.getTargetPosition())) {
        }
        return Math.abs(FL.getTargetPosition()) - Math.abs(FL.getCurrentPosition() * proportionalValue);
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
                FR.setTargetPosition(target);
                BL.setTargetPosition(-target);
                BR.setTargetPosition(target);
                break;

            case RIGHTTURN:
                FL.setTargetPosition(target);
                FR.setTargetPosition(-target);
                BL.setTargetPosition(target);
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
                BL.setPower(power);
                BR.setPower(-power);
                break;

            case RIGHTSTRAFE:
                FL.setPower(-power);
                FR.setPower(power);
                BL.setPower(-power);
                BR.setPower(power);
                break;

            case LEFTTURN:
                FL.setPower(-power);
                FR.setPower(power);
                BL.setPower(-power);
                BR.setPower(power);
                break;

            case RIGHTTURN:
                FL.setPower(power);
                FR.setPower(-power);
                BL.setPower(power);
                BR.setPower(-power);
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
        this.clamp.setPosition(0);
    }

    public void openHingeAuton() {
        this.clamp.setPosition(1);
    }

    public void openServo() {
        this.foundationLeft.setPosition(0);
        this.foundationRight.setPosition(1);
    }

    public void closeServo() {
        this.foundationLeft.setPosition(1);
        this.foundationRight.setPosition(0);
    }

    public void openServoAuton() {
        this.foundationLeft.setPosition(0);
        this.foundationRight.setPosition(1);
    }

    public void closeServoAuton() {
        this.foundationLeft.setPosition(1);
        this.foundationRight.setPosition(0);
    }

    public void raiseLift(double height) {
        this.lift.setDirection(DcMotorSimple.Direction.FORWARD);
        this.lift.setTargetPosition(cmDistance(height));
        this.lift.setPower(0.8);
        this.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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
            power = .25;
        } else if (diff < 300) {
            power = .35;
        } else if (diff < 500) {
            power = .4;
        } else if (diff < 750) {
            power = .5;
        }
        this.drive(power);
    }


    public void tapeExtend(int target, double power) {
        this.tape.setTargetPosition(target);
        this.tape.setPower(power);
        this.tape.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if ((Math.abs(this.tape.getCurrentPosition() - this.tape.getTargetPosition()) < 25)) {
            this.tape.setPower(0);
            this.command++;
        }
    }

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





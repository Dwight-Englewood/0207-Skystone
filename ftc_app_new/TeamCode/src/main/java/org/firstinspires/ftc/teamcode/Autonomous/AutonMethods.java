package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;

import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Hardware.Movement;

public class AutonMethods {
    public static DcMotor BL, BR, FL, FR, lift;
    public static Servo LSERV, RSERV, clamp;
    HardwareMap map;
    Telemetry tele;

    public ColorSensor color_sensor;

    final double proportionalValue = 0.000005;
    public int command;
    private int originTick;

    //double error = 180 - gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    //Double turnSpeed = 0.5;
    //Integer angle = -45;
    public static BNO055IMU gyro;
    BNO055IMU.Parameters parameters;
    Orientation angles;

    public AutonMethods() {
        command = 0;
    }

    public void init(HardwareMap map, Telemetry tele, boolean auton) {
        this.map = map;
        this.tele = tele;

        color_sensor = this.map.get(ColorSensor.class, "Col");

        BR = this.map.get(DcMotor.class, "BR");
        BL = this.map.get(DcMotor.class, "BL");
        FL = this.map.get(DcMotor.class, "FL");
        FR = this.map.get(DcMotor.class, "FR");
        lift = this.map.get(DcMotor.class, "Lift");

        clamp = this.map.get(Servo.class, "clamp");
        LSERV = this.map.get(Servo.class, "LSERV");
        RSERV = this.map.get(Servo.class, "RSERV");

        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.FORWARD);
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        FR.setDirection(DcMotorSimple.Direction.FORWARD);

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

    public void openServo() {
        this.LSERV.setPosition(1);
        this.RSERV.setPosition(1);
    }

    public void closeServo() {
        this.LSERV.setPosition(0);
        this.RSERV.setPosition(0);
    }

    public boolean adjustHeading(int targetHeading) {
        double curHeading = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        double headingError;
        headingError = targetHeading - curHeading;
        double driveScale = headingError;
        this.changeRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (headingError < -0.3)
            driveScale = -0.15;
        else if (headingError > 0.3)
            driveScale = 0.15;
        else {
            driveScale = 0;
            this.drive(Movement.LEFTTURN, driveScale);
            return true;
        }
        this.drive(Movement.LEFTTURN, driveScale);
        return false;
    }

    public double strafeVal(double target){
        return (target*1.2);
    }

    public void runToTarget(Movement movementEnum, double target, boolean strafe) {
        if (strafe) {
            this.autonDrive(movementEnum, cmDistance(strafeVal(target)));
        } else {
            this.autonDrive(movementEnum, cmDistance(target));
        }
        this.scalePower();
        this.changeRunMode(DcMotor.RunMode.RUN_TO_POSITION);

//        if ((Math.abs(FL.getCurrentPosition() + 25) >= Math.abs(FL.getTargetPosition()) &&
//                Math.abs(FL.getCurrentPosition() - 25) <= Math.abs(FL.getTargetPosition()))) {
        tele.addData("Delta", Math.abs(FL.getCurrentPosition() - FL.getTargetPosition()));
        if (Math.abs(FL.getCurrentPosition() - FL.getTargetPosition()) < 25) {
            autonDrive(movementEnum.STOP, 0);
            tele.update();
            this.command++;
        }
    }

    public void encoderReset() {
        this.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.command++;
    }

    public void closeClampAuton(){
        this.clamp.setPosition(0);
        this.sleepFunc(1000);
        this.encoderReset();
    }

    public void openClampAuton(){
        this.clamp.setPosition(1);
        this.sleepFunc(1000);
        this.encoderReset();
    }

    public void closeServoAuton(){
        this.LSERV.setPosition(0);
        this.RSERV.setPosition(0);
        this.sleepFunc(1000);
        this.encoderReset();
    }

    public void openServoAuton(){
        this.LSERV.setPosition(1);
        this.RSERV.setPosition(1);
        this.sleepFunc(1000);
        this.encoderReset();
    }

    public void sleepFunc(long time ) {
        try {
            Thread.sleep(time);
        } catch (InterruptedException E) {
            tele.addLine("Sleep Failed");
        }
    }

    public void scalePower() {
        int target = FL.getTargetPosition();
        int current = FL.getCurrentPosition();
        int diff = Math.abs(target - current);
        int originDiff = Math.abs(this.originTick - current);
        double power;

        if (originDiff < 75) {
            power = .25;
        } else if (originDiff < 250) {
            power = .3;
        } else if (originDiff < 400) {
            power = .375;
        } else {
            power = .45;
        }

        if (diff < 100) {
            power = .25;
        } else if (diff < 300) {
            power = .35;
        } else if (diff < 500) {
            power = .375;
        } else if (diff < 750) {
            power = .45;
        }

        this.drive(power);
    }

    private int cmDistance(double distance) {
        final double wheelCirc = 31.9185813;
        final double gearMotorTick = 537.6; //neverrest orbital 20 = 537.6 counts per revolution
        //1:1 gear ratio so no need for multiplier
        return (int) (gearMotorTick * (distance / wheelCirc));
    }

}



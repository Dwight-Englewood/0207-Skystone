package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;

import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Hardware.Movement;


public class Boot {
    public static DcMotor
            BL,
            BR,
            FL,
            FR
                    //         lift,
                    //         intakeL,
                    //         intakeR
                    ;
    public static Servo
            LSERV,
            RSERV;

    HardwareMap map;
    Telemetry tele;

    public ColorSensor color_sensor;

    final double proportionalValue = 0.000005;

    //double error = 180 - gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    //Double turnSpeed = 0.5;
    //Integer angle = -45;
    public static BNO055IMU gyro;
    BNO055IMU.Parameters parameters;
    Orientation angles;

    public Boot() {
    }

    public void init(HardwareMap map, Telemetry tele, boolean auton) {
        this.map = map;
        this.tele = tele;

        color_sensor = this.map.get(ColorSensor.class, "Col");

        BR = this.map.get(DcMotor.class, "BR");
        BL = this.map.get(DcMotor.class, "BL");
        FL = this.map.get(DcMotor.class, "FL");
        FR = this.map.get(DcMotor.class, "FR");
        //      lift = this.map.get(DcMotor.class, "Lift");
        //     intakeL = this.map.get(DcMotor.class, "intakeL");
        //      intakeR = this.map.get(DcMotor.class, "intakeR");

        LSERV = this.map.get(Servo.class, "LSERV");
        RSERV = this.map.get(Servo.class, "RSERV");

        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.FORWARD);
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        FR.setDirection(DcMotorSimple.Direction.FORWARD);
        //      lift.setDirection(DcMotorSimple.Directiofn.FORWARD);
        //      intakeL.setDirection((DcMotorSimple.Direction.FORWARD));
        //      intakeR.setDirection((DcMotorSimple.Direction.REVERSE));

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

    public void notKevinDrive(double leftStick_y, double leftStick_x, double leftTrigger, double rightTrigger) {
        if (leftTrigger > .3) {
            drive(Movement.LEFTSTRAFE, leftTrigger * 0.75);
            return;
        }
        if (rightTrigger > .3) {
            drive(Movement.RIGHTSTRAFE, rightTrigger * 0.75);
            return;
        }

        //   leftStick *= i;
        //  rightStick *= i;
        FL.setPower(leftStick_y);
        FR.setPower(leftStick_y);
        BL.setPower(-leftStick_y);
        BR.setPower(-leftStick_y);

        FL.setPower(leftStick_x);
        FR.setPower(-leftStick_x);
        BL.setPower(-leftStick_x);
        BR.setPower(leftStick_x);
    }

    public void tankDrive(double leftStick, double rightStick, double leftTrigger, double rightTrigger, boolean invert, boolean brake) {
        double i = invert ? 0.4 : 0.7;
        //  double s = sickoMode ? 0.4 : 1;

        if (leftTrigger > .3) {
            drive(Movement.LEFTSTRAFE, leftTrigger * i);
            return;
        }

        if (rightTrigger > .3) {
            drive(Movement.RIGHTSTRAFE, rightTrigger * i);
            return;
        }
        leftStick *= i;
        rightStick *= i;

        FL.setPower(rightStick);
        FR.setPower(leftStick);
        BL.setPower(-rightStick);
        BR.setPower(-leftStick);
    }
    //TODO fix the the driver values and restrict the motor values
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

}



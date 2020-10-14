package org.firstinspires.ftc.teamcode.Hardware;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class
DeuxBoot{
    public DcMotor
            BL, BR, FL, FR, lift, intakeL, intakeR;

    public Servo
            closer, hinger, foundationLeft, foundationRight,
            leftBlue, leftPurp, rightBlue, rightPurp, hinge, spinner, grabber, extend;

    public float curHeading;

    HardwareMap map;
    Telemetry tele;

    public ElapsedTime runtime = new ElapsedTime();
    public static BNO055IMU gyro;

    public DeuxBoot() {
    }

    /**
     * @param map creates an object on phone
     */
    public void initNew(HardwareMap map) {
        this.map = map;

        BR = this.map.get(DcMotor.class, "BR");
        BL = this.map.get(DcMotor.class, "BL");
        FL = this.map.get(DcMotor.class, "FL");
        FR = this.map.get(DcMotor.class, "FR");
        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.FORWARD);
        FL.setDirection(DcMotorSimple.Direction.FORWARD);
        FR.setDirection(DcMotorSimple.Direction.REVERSE);

        lift = this.map.get(DcMotor.class, "lift");
        lift.setDirection(DcMotorSimple.Direction.FORWARD);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeL = this.map.get(DcMotor.class, "intakeL");
        intakeL.setDirection((DcMotorSimple.Direction.REVERSE));
        intakeL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeR = this.map.get(DcMotor.class, "intakeR");
        intakeR.setDirection((DcMotorSimple.Direction.REVERSE));
        intakeR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        foundationLeft = this.map.get(Servo.class, "fleft");
        foundationRight = this.map.get(Servo.class, "fright");

        leftBlue = this.map.get(Servo.class, "leftBlue");
        leftPurp = this.map.get(Servo.class, "leftPurp");
        rightBlue = this.map.get(Servo.class, "rightBlue");
        rightPurp = this.map.get(Servo.class, "rightPurp");

        hinge = this.map.get(Servo.class, "hinge");
        spinner = this.map.get(Servo.class, "spinner");
        grabber = this.map.get(Servo.class, "grabber");
        extend = this.map.get(Servo.class, "extend");

        this.changeRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
     *
     * Changes the motor run mode
     *
     * @param runMode the new runmode.
     */
    public void changeRunMode(DcMotor.RunMode runMode) {
        BL.setMode(runMode);
        BR.setMode(runMode);
        FL.setMode(runMode);
        FR.setMode(runMode);
    }

    /**
     *
     *Sets the motor power levels according to inputs from the controllers Two stick
     *
     * @param leftStick_y the y power values from the left stick on the controllers (-1,1)
     * @param rightStick_y the y power values from the right stick on the controllers (-1,1)
     * @param leftTrigger the power values from the left trigger
     * @param rightTrigger the power values from the right trigger
     */
    public void drive(double leftStick_y, double rightStick_y, double leftTrigger, double rightTrigger) {
        if (leftTrigger > .3) {
            drive(Movement.LEFTSTRAFE, leftTrigger * 0.75);
            return;
        }
        if (rightTrigger > .3) {
            drive(Movement.RIGHTSTRAFE, rightTrigger * 0.75);
            return;
        }
        FL.setPower(leftStick_y);
        FR.setPower(rightStick_y);
        BL.setPower(leftStick_y);
        BR.setPower(rightStick_y);
    }

    /**
     *
     * Sets the motor power levels according to inputs from the controllers. Uses one stick.
     *
     * @param leftStick_y The y power level from the left stick on the controller (-1,1)
     * @param leftStick_x The x power level from the left stick on the controller(-1,1)
     * @param leftTrigger The power levels from the left Trigger
     * @param rightTrigger The power levels from the left Trigger.
     */
    public void notKevinDrive(double leftStick_y, double leftStick_x, double leftTrigger, double rightTrigger, double speed) {
        leftStick_y *= -1;
        if (leftTrigger > .3) {
            drive(Movement.LEFTSTRAFE, leftTrigger);
            return;
        }
        if (rightTrigger > .3) {
            drive(Movement.RIGHTSTRAFE, rightTrigger);
            return;
        }
        if (Math.abs(leftStick_y) > .5) {
            FL.setPower(-leftStick_y * speed);
            FR.setPower(-leftStick_y * speed);
            BL.setPower(-leftStick_y * speed);
            BR.setPower(-leftStick_y * speed);
            return;
        }

        FL.setPower(-leftStick_x * speed);
        FR.setPower(leftStick_x * speed);
        BL.setPower(-leftStick_x * speed);
        BR.setPower(leftStick_x * speed);
    }

    public void kevinDrive(double leftStick_y, double leftStick_x, double leftTrigger, double rightTrigger){
        leftStick_y *= -1;
        if (leftTrigger > .3) {
            drive(Movement.LEFTSTRAFE, leftTrigger);
            return;
        }
        if (rightTrigger > .3) {
            drive(Movement.RIGHTSTRAFE, rightTrigger);
            return;
        }
        if (Math.abs(leftStick_y) > .5) {
            FL.setPower(-leftStick_y * - returnHeading());
            FR.setPower(-leftStick_y * + returnHeading());
            BL.setPower(-leftStick_y * + returnHeading());
            BR.setPower(-leftStick_y * - returnHeading());
            return;
        }

        FL.setPower(-leftStick_x * - returnHeading());
        FR.setPower(leftStick_x * + returnHeading());
        BL.setPower(-leftStick_x * + returnHeading());
        BR.setPower(leftStick_x * - returnHeading());
    }

    public void tankDrive(double leftStickX, double leftStickY, double rightStickX, double leftTrigger, double rightTrigger) {
        if (leftTrigger > .3) {
            drive(Movement.LEFTTURN, leftTrigger);
            return;
        }
        if (rightTrigger > .3) {
            drive(Movement.RIGHTTURN, rightTrigger);
            return;
        }
        if (gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle > 45) {
            FL.setPower(leftStickY - leftStickX - rightStickX);
            FR.setPower(leftStickY + leftStickX + rightStickX);
            BL.setPower(leftStickY + leftStickX - rightStickX);
            BR.setPower(leftStickY - leftStickX + rightStickX);
        }
    }


    public double returnHeading(){
        return (Math.sin(gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle));
    }
    /**
     *
     * Sets motor power levels to specified power levels..
     *
     * @param movement Direction of movement.
     * @param power level of power that motor will be set to.
     */
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
}



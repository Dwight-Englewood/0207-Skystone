package org.firstinspires.ftc.teamcode.Hardware;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class
DeuxBoot{
    public DcMotor
            BL, BR, FL, FR, lift, intakeL, intakeR;

    public Servo
            closer, hinger, foundationLeft, foundationRight,
            leftBlue, leftPurp, rightBlue, rightPurp, hinge, spinner, grabber;

    HardwareMap map;
    Telemetry tele;

    public ElapsedTime runtime = new ElapsedTime();


    public DeuxBoot() {
    }

    /**
     * @param map creates an object on phone
     */
    public void init(HardwareMap map) {
        this.map = map;

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

        BR.setDirection(DcMotorSimple.Direction.FORWARD);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        FR.setDirection(DcMotorSimple.Direction.FORWARD);

        closer.setPosition(1);
        this.changeRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void initNew(HardwareMap map) {
        this.map = map;

        BR = this.map.get(DcMotor.class, "BR");
        BL = this.map.get(DcMotor.class, "BL");
        FL = this.map.get(DcMotor.class, "FL");
        FR = this.map.get(DcMotor.class, "FR");
        BR.setDirection(DcMotorSimple.Direction.FORWARD);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        FR.setDirection(DcMotorSimple.Direction.REVERSE);

        lift = this.map.get(DcMotor.class, "lift");
        lift.setDirection(DcMotorSimple.Direction.FORWARD);

        intakeL = this.map.get(DcMotor.class, "intakeL");
        intakeL.setDirection((DcMotorSimple.Direction.REVERSE));

        intakeR = this.map.get(DcMotor.class, "intakeR");
        intakeR.setDirection((DcMotorSimple.Direction.REVERSE));

        /*foundationLeft = this.map.get(Servo.class, "fleft");
        foundationRight = this.map.get(Servo.class, "fright");

        leftBlue = this.map.get(Servo.class, "leftBlue");
        leftPurp = this.map.get(Servo.class, "leftPurp");
        rightBlue = this.map.get(Servo.class, "rightBlue");
        rightPurp = this.map.get(Servo.class, "rightPurp");

        hinge = this.map.get(Servo.class, "hinge");
        spinner = this.map.get(Servo.class, "spinner");
        grabber = this.map.get(Servo.class, "grabber");

         */

        this.changeRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
    public void notKevinDrive(double leftStick_y, double leftStick_x, double leftTrigger, double rightTrigger) {
        if (leftTrigger > .3) {
            drive(Movement.LEFTSTRAFE, leftTrigger * 0.75);
            return;
        }
        if (rightTrigger > .3) {
            drive(Movement.RIGHTSTRAFE, rightTrigger * 0.75);
            return;
        }

        FL.setPower(leftStick_y);
        FR.setPower(leftStick_y);
        BL.setPower(leftStick_y);
        BR.setPower(leftStick_y);

        FL.setPower(-leftStick_x);
        FR.setPower(leftStick_x);
        BL.setPower(-leftStick_x);
        BR.setPower(leftStick_x);
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



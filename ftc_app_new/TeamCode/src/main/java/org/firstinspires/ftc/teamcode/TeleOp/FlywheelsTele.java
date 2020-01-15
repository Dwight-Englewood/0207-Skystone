package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Hardware.DeuxBoot;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Tele-op",group="TeleOp")
public class FlywheelsTele extends OpMode {
    // Declare OpMode members.
    private ElapsedTime timer = new ElapsedTime();
    DeuxBoot robot = new DeuxBoot();
    double speed = 1;
    boolean buttonAheld = false;
    boolean closerClosed = true;
    boolean buttonBheld = false;
    boolean hingedClosed = true;
    boolean intakeHeld = false;
    boolean clampClosed = true;

    @Override
    public void init() {
        robot.init(hardwareMap, telemetry, false);
        telemetry.addData("Status", "Initialized");

        /*robot.BR.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.BL.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.FL.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.FR.setDirection(DcMotorSimple.Direcion.REVERSE);
         */

        robot.lift.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.intakeL.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.intakeR.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.closer.setPosition(1);
        robot.blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_LAVA_PALETTE);
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        robot.blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
        timer.reset();
    }

    /*
     * Code to run REPEATEDLY after the r hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        robot.notKevinDrive(gamepad1.left_stick_y * speed, gamepad1.left_stick_x * speed, gamepad1.left_trigger * speed, gamepad1.right_trigger * speed);
        robot.lift.setPower(gamepad2.right_stick_y);
 //       robot.intakeL.setPower(gamepad2.left_stick_y * 1);
 //       robot.intakeR.setPower(gamepad2.left_stick_y * 1);

        if (robot.intakeL.getPower() != 0) {
            robot.blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_OCEAN_PALETTE);
        } else if (robot.intakeL.getPower() == 0) {
            robot.blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_LAVA_PALETTE);
        }

        if (gamepad1.b) {
            speed = 0.5;
            robot.blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.GOLD);
        } else if (gamepad1.a) {
            speed = 1;
        } else if (gamepad1.y) {
            speed = -1;
            robot.blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
        }

        if (gamepad1.left_bumper && !intakeHeld) {
            intakeHeld = true;
            timer.reset();
            if (clampClosed) {
                clampClosed = false;
                robot.intakeL.setPower(1);
                robot.intakeR.setPower(1);
                if (timer.milliseconds() > 3000 && robot.intakeL.getPower() == 0 && robot.intakeR.getPower() == 0) {
                    robot.closer.setPosition(0);
                }
            } else {
                closerClosed = true;
                robot.intakeL.setPower(-1);
                robot.intakeR.setPower(-1);
            }
        }

        if (!gamepad1.left_bumper) {
            robot.intakeL.setPower(0);
            robot.intakeR.setPower(0);
            robot.closer.setPosition(1);
            intakeHeld = false;
        }
/*
        if (gamepad1.left_bumper) {
            robot.intakeL.setPower(1);
            robot.intakeL.setPower(1);
        } else if (gamepad1.right_bumper) {
            robot.intakeL.setPower(-1);
            robot.intakeL.setPower(-1);
        } else {
            robot.intakeL.setPower(0);
            robot.intakeL.setPower(0);
        }

 */
/*
        if (gamepad2.a && !buttonAheld) {
            buttonAheld = true;
            if (closerClosed) {
                closerClosed = false;
                robot.closer.setPosition(1);
            } else {
                closerClosed = true;
                robot.closer.setPosition(0);
            }
        }

        if (!gamepad2.a) {
            buttonAheld = false;
        }

 */

        if (gamepad2.b && !buttonBheld) {
            buttonBheld = true;
            if (hingedClosed) {
                hingedClosed = false;
                robot.hinger.setPosition(0.75);
            } else {
                hingedClosed = true;
                robot.hinger.setPosition(0);
            }
        }

        if (!gamepad2.b) {
            buttonBheld = false;
        }

        if (gamepad2.dpad_up) {
            robot.foundationLeft.setPosition(0);
            robot.foundationRight.setPosition(1);
        }

        if (gamepad2.dpad_down) {
            robot.foundationLeft.setPosition(0.55);
            robot.foundationRight.setPosition(0.35);
        }
    }


    /*
     * Code to run ONCE after the driver hits STOP
     */


    @Override
    public void stop () {
    }
}

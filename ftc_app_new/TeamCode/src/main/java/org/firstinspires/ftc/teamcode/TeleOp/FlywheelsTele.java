package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.DeuxBoot;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="New Tele-op",group="TeleOp")
public class FlywheelsTele extends OpMode {
    // Declare OpMode members.
    private ElapsedTime timer = new ElapsedTime();
    DeuxBoot robot = new DeuxBoot();
    double speed = 1;
    boolean valo = true;

    @Override
    public void init() {
        robot.init(hardwareMap, telemetry, false);
        telemetry.addData("Status", "Initialized");

        /*robot.BR.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.BL.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.FL.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.FR.setDirection(DcMotorSimple.Direction.REVERSE);

         */

        robot.lift.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.intakeL.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.intakeR.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        timer.reset();
    }

    /*
     * Code to run REPEATEDLY after the r hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        robot.notKevinDrive(gamepad1.left_stick_y * speed, gamepad1.left_stick_x * speed, gamepad1.left_trigger * speed, gamepad1.right_trigger * speed);
        robot.lift.setPower(gamepad2.right_stick_y);
        robot.intakeL.setPower(gamepad2.left_stick_y * 0.8);
        robot.intakeR.setPower(gamepad2.left_stick_y * 0.8);

        if (gamepad1.b) {
            speed = 0.5;
        } else if (gamepad1.a) {
            speed = 1;
        } else if (gamepad1.y) {
            speed = -1;
        }
/*
        if (gamepad2.a) { //open
            robot.closer.setPosition(1);
        }

        if (gamepad2.b) { //close
            robot.closer.setPosition(0);
        }
     */
        if (gamepad2.a && valo == true) {
            robot.closer.setPosition(1);
            valo = false;
        } else if (gamepad2.a && valo == false) {
            robot.closer.setPosition(0);
            valo = true;
        }

        if (gamepad2.x) { //open
            robot.hinger.setPosition(0.75);
        }

        if (gamepad2.y) { //close
            robot.hinger.setPosition(0);
        }

        if (gamepad2.dpad_up) {
            robot.foundationLeft.setPosition(0);
            robot.foundationRight.setPosition(1);
        }

        if (gamepad2.dpad_down) {
            robot.foundationLeft.setPosition(1);
            robot.foundationRight.setPosition(0);
        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */


    @Override
    public void stop () {
    }
}

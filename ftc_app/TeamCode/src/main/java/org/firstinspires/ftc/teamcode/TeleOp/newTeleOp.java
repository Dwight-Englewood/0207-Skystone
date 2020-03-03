package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.Hardware.DeuxBoot;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="New Tele-Op",group="TeleOp")
public class newTeleOp extends OpMode {
    DeuxBoot robot = new DeuxBoot();
    double speed;
    boolean buttonAheld = false;
    boolean grabberClosed = true;
    boolean buttonBheld = false;
    boolean hingedClosed = true;
    boolean buttonXheld = false;
    boolean spinnerClosed = false;

    @Override
    public void init() {
        robot.initNew(hardwareMap);
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        robot.runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the r hits PLAY but before they hit STOP
     */

    @Override
    public void loop() {
        robot.notKevinDrive(gamepad1.left_stick_y * speed,
                gamepad1.left_stick_x * speed,
                gamepad1.left_trigger * speed,
                gamepad1.right_trigger * speed);

        robot.lift.setPower(gamepad2.right_stick_y);
        robot.intakeL.setPower(gamepad2.left_stick_y*0.75);
        robot.intakeR.setPower(-gamepad2.left_stick_y*0.75);

        if (gamepad1.b) {
            speed = 0.5;
        } else if (gamepad1.a) {
            speed = 1;
        } else if (gamepad1.y) {
            speed = -1;
        }
        telemetry.addData("Speed", speed);

        if (gamepad2.y){
            robot.foundationLeft.setPosition(1);
            robot.foundationRight.setPosition(1);
        } else {
            robot.foundationLeft.setPosition(0);
            robot.foundationRight.setPosition(0);
        }

        if (gamepad2.a && !buttonAheld) {
            buttonAheld = true;
            if (grabberClosed) {
                grabberClosed = false;
                robot.grabber.setPosition(1);
            } else {
                grabberClosed = true;
                robot.grabber.setPosition(0);
            }
        }

        if (!gamepad2.a) {
            buttonAheld = false;
        }

        if (gamepad2.b && !buttonBheld) {
            buttonBheld = true;
            if (hingedClosed) {
                hingedClosed = false;
                robot.hinge.setPosition(1);
                robot.spinner.setPosition(1);
            } else {
                hingedClosed = true;
                robot.hinge.setPosition(0);
                robot.spinner.setPosition(0);
            }
        }

        if (!gamepad2.b) {
            buttonBheld = false;
        }

        if (gamepad2.x && !buttonXheld) {
            buttonXheld = true;
            if (spinnerClosed) {
                spinnerClosed = false;
                robot.spinner.setPosition(0);
            } else {
                spinnerClosed = true;
                robot.spinner.setPosition(0.35);
            }
        }

        if (!gamepad2.x) {
            buttonXheld = false;
        }
    }


    /*
     * Code to run ONCE after the driver hits STOP
     */

    @Override
    public void stop () {
    }
}

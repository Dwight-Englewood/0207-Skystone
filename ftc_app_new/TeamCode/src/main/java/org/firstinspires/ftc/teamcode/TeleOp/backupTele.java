package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.Hardware.DeuxBoot;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Backup Tele-op",group="TeleOp")
public class backupTele extends OpMode {
    DeuxBoot robot = new DeuxBoot();
    double speed;
    boolean buttonAheld = false;
    boolean closerClosed = true;
    boolean buttonBheld = false;
    boolean hingedClosed = true;

    @Override
    public void init() {
        robot.init(hardwareMap);
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
        robot.drive(gamepad1.left_stick_y * speed,
                gamepad1.left_stick_x * speed,
                gamepad1.left_trigger * speed,
                gamepad1.right_trigger * speed);

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

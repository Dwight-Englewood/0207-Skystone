package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.Hardware.DeuxBoot;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="New Tele-Op",group="TeleOp")
public class newBotTele extends OpMode {
    DeuxBoot robot = new DeuxBoot();
    double speed;

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

        robot.liftL.setPower(gamepad2.right_stick_y);
        robot.liftR.setPower(gamepad2.right_stick_y);

        robot.intakeL.setPower(gamepad2.left_stick_y);
        robot.intakeR.setPower(gamepad2.left_stick_y);

        if (gamepad1.b) {
            speed = 0.5;
        } else if (gamepad1.a) {
            speed = 1;
        } else if (gamepad1.y) {
            speed = -1;
        }
        telemetry.addData("Speed", speed);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */

    @Override
    public void stop () {
    }
}

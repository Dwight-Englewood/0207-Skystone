package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.Hardware.DeuxBoot;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="New Tele-Op",group="TeleOp")
public class newTeleOp extends OpMode {
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

        robot.lift.setPower(gamepad2.right_stick_y);

        robot.intakeL.setPower(gamepad2.left_stick_y);
        robot.intakeR.setPower(-gamepad2.left_stick_y);

        if (gamepad1.b) {
            speed = 0.5;
        } else if (gamepad1.a) {
            speed = 1;
        } else if (gamepad1.y) {
            speed = -1;
        }
        telemetry.addData("Speed", speed);

        /*if (gamepad2.x){
            robot.foundationLeft.setPosition(1);
            robot.foundationRight.setPosition(1);
        } else {
            robot.foundationLeft.setPosition(0);
            robot.foundationRight.setPosition(0);
        }

        if (gamepad2.dpad_up){
            robot.hinge.setPosition(1);
            robot.spinner.setPosition(1);
            robot.grabber.setPosition(1);

        } else if (gamepad2.dpad_down){
            robot.hinge.setPosition(0);
            robot.spinner.setPosition(0);
            robot.grabber.setPosition(0);
        }

        if (gamepad2.dpad_right){
            robot.leftBlue.setPosition(1);
            robot.leftPurp.setPosition(1);
            robot.rightBlue.setPosition(1);
            robot.rightPurp.setPosition(1);

        } else if (gamepad2.dpad_left){
            robot.leftBlue.setPosition(0);
            robot.leftPurp.setPosition(0);
            robot.rightBlue.setPosition(0);
            robot.rightPurp.setPosition(0);
        }

         */
    }



    /*
     * Code to run ONCE after the driver hits STOP
     */

    @Override
    public void stop () {
    }
}

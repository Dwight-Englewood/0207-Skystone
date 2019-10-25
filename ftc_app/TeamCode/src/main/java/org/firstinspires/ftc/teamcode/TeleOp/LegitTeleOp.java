package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Boot;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Green Wheels",group="TeleOp")
//@Disabled
public class LegitTeleOp extends OpMode {
    // Declare OpMode members.
    private ElapsedTime timer = new ElapsedTime();
    Boot robot = new Boot();
    boolean wabbo = false;

    @Override
    public void init() {
        robot.init(hardwareMap, telemetry, false);
        telemetry.addData("Status", "Initialized");
        robot.clamp.setPosition(0.6);
        robot.clawTurn.setPosition(0);
   //     robot.clawTurn.setPosition(1);
   //     robot.armTurn.setPosition(1);
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
        if (gamepad1.y) {
            wabbo = false;
        }

        robot.tankDrive(gamepad1.left_stick_y, gamepad1.right_stick_y, gamepad1.left_trigger, gamepad1.right_trigger, wabbo, false);
        robot.lift.setPower(gamepad2.right_stick_y * 0.7);
        robot.intakeL.setPower(gamepad2.left_stick_y * 0.7);
        robot.intakeR.setPower(gamepad2.left_stick_y * 0.7);

        if (gamepad2.right_bumper) {
            robot.clamp.setPosition(0);
        } else  {
            robot.clamp.setPosition(0.6);
        }

        if (gamepad2.left_bumper) {
            robot.clawTurn.setPosition(1);
        } else {
             robot.clawTurn.setPosition(0);
        }



        /*
         * Code to run ONCE after the driver hits STOP
         */

    }
    @Override
    public void stop() {
    }

}
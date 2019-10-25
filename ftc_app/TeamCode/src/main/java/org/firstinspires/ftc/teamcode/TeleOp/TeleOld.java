package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Boot;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="2 servos 1 motor",group="TeleOp")
//@Disabled
public class TeleOld extends OpMode {
    // Declare OpMode members.
    private ElapsedTime timer = new ElapsedTime();
    Boot robot = new Boot();
    boolean wabbo = false;

    @Override
    public void init() {
        robot.init(hardwareMap, telemetry, false);
        telemetry.addData("Status", "Initialized");
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
        robot.tankDrive(gamepad1.left_stick_y, gamepad1.right_stick_y, gamepad1.left_trigger,gamepad1.right_trigger,wabbo, false);
        robot.lift.setPower(gamepad2.right_stick_y*0.7);

        if (gamepad2.x) {
            robot.clamp.setPosition(1);
        }

        if (gamepad2.y) {
            robot.clamp.setPosition(0);
        }
        
        telemetry.addLine("G2Y: Close Clamp");
        telemetry.addLine("G2X: Open Clamp");
    }



    /*
     * Code to run ONCE after the driver hits STOP
     */


    @Override
    public void stop() {
    }

}
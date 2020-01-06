package org.firstinspires.ftc.teamcode.TeleOp.Old;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.DeuxBoot;
@Disabled
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Tele-op (Cool Bot)",group="TeleOp")
public class LegitTeleOp extends OpMode {
    // Declare OpMode members.
    private ElapsedTime timer = new ElapsedTime();
    DeuxBoot robot = new DeuxBoot();
    double speed = 1;

    @Override
    public void init() {
        robot.init(hardwareMap, telemetry, false);
        telemetry.addData("Status", "Initialized");

        robot.BR.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.BL.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.FL.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.FR.setDirection(DcMotorSimple.Direction.REVERSE);

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
        robot.drive(gamepad1.left_stick_y * speed, gamepad1.left_stick_x * speed, gamepad1.left_trigger * speed, gamepad1.right_trigger * speed);
        robot.lift.setPower(gamepad2.right_stick_y);

        if (gamepad1.a) {
            speed = 0.5;
        } else if (gamepad1.b) {
            speed = 1;
        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */


    @Override
    public void stop () {
    }
}

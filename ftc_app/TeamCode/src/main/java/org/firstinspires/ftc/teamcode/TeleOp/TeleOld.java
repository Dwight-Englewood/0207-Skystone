package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Boot;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="1 servo 1 motor (Works)",group="TeleOp")
//@Disabled
public class TeleOld extends OpMode {
    // Declare OpMode members.
    private ElapsedTime timer = new ElapsedTime();
    Boot robot = new Boot();
    boolean wabbo = false;

    public static DcMotor lift;
    public static Servo clamp;

    @Override
    public void init() {
        robot.init(hardwareMap, telemetry, false);
        telemetry.addData("Status", "Initialized");

        lift = this.hardwareMap.get(DcMotor.class, "Lift");
        clamp = this.hardwareMap.get(Servo.class, "clamp");

        lift.setDirection(DcMotorSimple.Direction.FORWARD);
        clamp.setPosition(0);

        robot.color_sensor.enableLed(false);

        robot.BR.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.BL.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.FL.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.FR.setDirection(DcMotorSimple.Direction.REVERSE);
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
        this.lift.setPower(gamepad2.right_stick_y);

        if (gamepad2.y) {
            this.clamp.setPosition(1);
        }

        if (gamepad2.x) {
            this.clamp.setPosition(0);
        }
        telemetry.addLine("G2X: Close Clamp");
        telemetry.addLine("G2Y: Open Clamp");
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */


    @Override
    public void stop() {
    }

}
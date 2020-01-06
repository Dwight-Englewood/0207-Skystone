package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Boot;
//@Disabled
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Tele-op ()",group="TeleOp")
public class AJTeleOld extends OpMode {
    // Declare OpMode members.
    private ElapsedTime timer = new ElapsedTime();
    Boot robot = new Boot();
    double speed = 1;

    @Override
    public void init() {
        robot.init(hardwareMap, telemetry, false);
        telemetry.addData("Status", "Initialized");

        robot.lift.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.tape.setDirection(DcMotorSimple.Direction.FORWARD);

        robot.BR.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.BL.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.FL.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.FR.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.openServo();
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

        if (gamepad2.dpad_up) {
            robot.tape.setPower(-1);
        } else if (gamepad2.dpad_down){
            robot.tape.setPower(1);
        } else {
            robot.tape.setPower(0);
        }

        if (gamepad1.b) {
                speed = 0.5;
            } else if (gamepad1.a) {
                speed = 1;
            }

            if (gamepad2.x) {
                robot.clamp.setPosition(0);
            }

            if (gamepad2.y) {
                robot.clamp.setPosition(1);
            }

            if (gamepad2.b) {
                robot.openServo();
            }

            if (gamepad2.a) {
                robot.closeServo();
            }

            telemetry.addLine("G2X: Close Clamp");
            telemetry.addLine("G2Y: Open Clamp");
            telemetry.addLine("G2B: Open Servo");
            telemetry.addLine("G2A: Close Servo");
        }

        /*
         * Code to run ONCE after the driver hits STOP
         */


        @Override
        public void stop () {
        }
}

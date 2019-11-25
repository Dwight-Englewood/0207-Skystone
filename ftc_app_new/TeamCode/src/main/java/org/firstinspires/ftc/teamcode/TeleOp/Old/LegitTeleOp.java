package org.firstinspires.ftc.teamcode.TeleOp.Old;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Boot;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Small Wheels (New)",group="TeleOp")
@Disabled
public class LegitTeleOp extends OpMode {
    // Declare OpMode members.
    private ElapsedTime timer = new ElapsedTime();
    Boot robot = new Boot();
    boolean wabbo = false;

    public static DcMotor lift, intakeL, intakeR;
    public static Servo clamp, clawTurn;

    @Override
    public void init() {
        robot.init(hardwareMap, telemetry, false);
        telemetry.addData("Status", "Initialized");

        lift = this.hardwareMap.get(DcMotor.class, "Lift");
        intakeL = this.hardwareMap.get(DcMotor.class, "intakeL");
        intakeR = this.hardwareMap.get(DcMotor.class, "intakeR");

        clamp = this.hardwareMap.get(Servo.class, "clamp");
        clawTurn = this.hardwareMap.get(Servo.class, "claw turn");

        lift.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeL.setDirection((DcMotorSimple.Direction.FORWARD));
        intakeR.setDirection((DcMotorSimple.Direction.REVERSE));

        this.clamp.setPosition(1);
        this.clawTurn.setPosition(1);

        this.clamp.setPosition(0.6);
        this.clawTurn.setPosition(0);
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
        this.lift.setPower(gamepad2.right_stick_y * 0.7);
        this.intakeL.setPower(gamepad2.left_stick_y * 1);
        this.intakeR.setPower(gamepad2.left_stick_y * 1);

        if (gamepad2.right_bumper) {
            this.clamp.setPosition(0.4);
        } else  {
            this.clamp.setPosition(0.6);
        }

        if (gamepad2.left_bumper) {
            this.clawTurn.setPosition(1);
        } else {
             this.clawTurn.setPosition(0);
        }

        /*
         * Code to run ONCE after the driver hits STOP
         */

    }
    @Override
    public void stop() {
    }

}
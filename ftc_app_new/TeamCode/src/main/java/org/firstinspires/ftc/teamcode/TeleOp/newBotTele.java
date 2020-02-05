package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Hardware.DeuxBoot;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="New Tele-Op",group="TeleOp")
public class newBotTele extends OpMode {
    // Declare OpMode members.
    private ElapsedTime timer = new ElapsedTime();
    DeuxBoot robot = new DeuxBoot();
    public static DcMotor liftRight, liftLeft;
    double speed = 1;

    @Override
    public void init() {
        robot.init(hardwareMap, telemetry, false);
        telemetry.addData("Status", "Initialized");

        liftRight = this.hardwareMap.get(DcMotor.class, "liftRight");
        liftLeft = this.hardwareMap.get(DcMotor.class, "liftLeft");


        robot.BR.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.BL.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.FL.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.FR.setDirection(DcMotorSimple.Direction.REVERSE);

        this.liftRight.setDirection(DcMotorSimple.Direction.FORWARD);
        this.liftLeft.setDirection(DcMotorSimple.Direction.FORWARD);
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
        this.liftRight.setPower(gamepad2.right_stick_y);
        this.liftLeft.setPower(gamepad2.right_stick_y);

        if (gamepad1.b) {
            speed = 0.5;
            robot.blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.GOLD);
        } else if (gamepad1.a) {
            speed = 1;
        } else if (gamepad1.y) {
            speed = -1;
            robot.blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
        }
    }


    /*
     * Code to run ONCE after the driver hits STOP
     */


    @Override
    public void stop () {
    }
}

package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Boot;
import org.firstinspires.ftc.teamcode.Hardware.*;

//hello

@Autonomous(name = "RedTrig (Test)", group = "Autonomous")
public class RedTrig extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DigitalChannel DigChannel;
    Boot robot = new Boot();

    int auto = 0;

    int center = 150;
    int left = 600;
    int right = 350;

    int centerBack = 1100;
    int leftBack = 800;
    int rightBack = 1750;

    public static Servo clamp;

    public void init() {
        robot.init(hardwareMap, telemetry, false);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        clamp = this.hardwareMap.get(Servo.class, "clamp");

        robot.BR.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.BL.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.FL.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.FR.setDirection(DcMotorSimple.Direction.FORWARD);

        robot.FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.openServo();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        switch (auto) {
            case 0:
                this.clamp.setPosition(1);
                robot.autonDriveUltimate(Movement.BACKWARD, 350, 0.5);
                if (Math.abs(robot.FL.getCurrentPosition()) >= Math.abs(robot.FL.getTargetPosition())){
                    auto++;
                }
                break;

            case 1:
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                auto++;
                break;

            case 2:
                robot.autonDriveUltimate(Movement.RIGHTSTRAFE, 1350, 0.5);
                if (Math.abs(robot.FL.getCurrentPosition()) >= Math.abs(robot.FL.getTargetPosition())){
                    auto++;
                }
                break;

            case 3:
                robot.closeServo();
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                auto++;
                break;

            case 4:
                robot.autonDriveUltimate(Movement.LEFTSTRAFE, 1350, 0.5);
                if (Math.abs(robot.FL.getCurrentPosition()) >= Math.abs(robot.FL.getTargetPosition())){
                    auto++;
                }
                break;

            case 5:
                robot.openServo();
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                auto++;
                break;

            case 6:
                robot.autonDriveUltimate(Movement.FORWARD, 1700, 0.5);
                if (Math.abs(robot.FL.getCurrentPosition()) >= Math.abs(robot.FL.getTargetPosition())){
                    auto++;
                }
                break;

            case 7:
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                break;
        }
        telemetry.addData("Case:", auto);
        telemetry.update();
    }
}

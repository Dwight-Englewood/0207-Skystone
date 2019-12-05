package org.firstinspires.ftc.teamcode.Autonomous.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Autonomous.Methods.AutonMethods;

@Disabled
@Autonomous(name = "BlueBoxWall [Test!]", group = "Autonomous")
public class BlueBoxWall extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DigitalChannel DigChannel;
    AutonMethods robot = new AutonMethods();

    int auto = 0;

    public static Servo clamp;
    public static DcMotor lift;


    public void init() {
        robot.init(hardwareMap, telemetry, false);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        robot.BR.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.BL.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.FL.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.FR.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.lift.setDirection(DcMotorSimple.Direction.FORWARD);

        robot.FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
            /*
            case 0:
                robot.runToTarget(Movement.FORWARD, 1200, 0.4);
                break;

            case 1:
                robot.closeClamp();
                break;

            case 2:
                robot.runToTarget(Movement.BACKWARD, 300, 0.5);
                break;

            case 3:
                robot.encoderReset();
                break;

            case 4:
                robot.runToTarget(Movement.RIGHTSTRAFE, 2550, 0.5);
                break;

            case 5:
                robot.openClamp();
                break;

            case 6:
                robot.runToTarget(Movement.BACKWARD, 200, 0.5);
                break;

            case 7:
                robot.encoderReset();
                break;

            case 8:
                robot.runToTarget(Movement.LEFTSTRAFE, 4900, 0.5);
                break;

            case 9:
                robot.encoderReset();
                break;

            case 10:
                robot.runToTarget(Movement.BACKWARD, 350, 0.5);
                break;

            case 11:
                robot.encoderReset();
                break;

            case 12:
                robot.runToTarget(Movement.FORWARD, 1200, 0.4);
                break;

            case 13:
                robot.encoderReset();
                break;

            case 14:
                robot.runToTarget(Movement.RIGHTSTRAFE, 700, 0.5);
                break;

            case 15:
                robot.closeClamp();
                break;

            case 16:
                robot.runToTarget(Movement.BACKWARD, 250, 0.5);
                break;

            case 17:
                robot.encoderReset();
                break;

            case 18:
                robot.runToTarget(Movement.RIGHTSTRAFE, 3650, 0.5);
                break;

            case 19:
                robot.openClamp();
                break;

            case 20:
                robot.runToTarget(Movement.FORWARD, 200, 0.5);
                break;

            case 21:
                robot.encoderReset();
                break;

            case 22:
                robot.runToTarget(Movement.LEFTSTRAFE, 1400, 0.5);
                break;

            case 23:
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                break;

             */
        }
        telemetry.addData("Case:", auto);
        telemetry.update();
    }
}

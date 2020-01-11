package org.firstinspires.ftc.teamcode.Autonomous.NewActiveAuton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Autonomous.Methods.NewAutonMethods;
import org.firstinspires.ftc.teamcode.Autonomous.Methods.SkystoneDetect;
import org.firstinspires.ftc.teamcode.Hardware.Movement;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

@Autonomous(name = "BBox", group = "Autonomous")
public class BBox extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DigitalChannel DigChannel;
    NewAutonMethods robot = new NewAutonMethods();
    SkystoneDetect detector = new SkystoneDetect();

    int stroll = 20;
    int count = 0;
    int block;

    public int left = 0;
    public int right = 0;
    public int middle = 0;
    public int notvis = 0;

    public static Servo clamp;

    public void init() {
        robot.init(hardwareMap, telemetry, false);
        telemetry.addData(">", "Init Vuforia.");
        telemetry.update();
        detector.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
/*
        robot.BR.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.BL.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.FL.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.FR.setDirection(DcMotorSimple.Direction.FORWARD);
 */

        robot.FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //    robot.tape.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
        robot.blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BREATH_BLUE);
        detector.start();
        runtime.reset();
    }
    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        switch (robot.command) {
            case 0:
                robot.runToTarget(Movement.FORWARD, 50, false);
                break;

            case 1:
                robot.encoderReset();
                break;

            case 2:
                robot.runToTarget(Movement.LEFTSTRAFE, 60, false);
                break;

            case 3:
                runtime.reset();
                robot.encoderReset();
                break;

            case 4:
                SkystoneDetect.Spot returnedloc = detector.getSkystonePosBlue(telemetry);
                switch (returnedloc) {
                    case LEFT:
                        left++;
                        break;

                    case RIGHT:
                        right++;
                        break;

                    case MIDDLE:
                        middle++;
                        break;

                    case NOTVISIBLE:
                        notvis++;
                        break;
                }

                if (runtime.milliseconds() > 2000) {
                    robot.command++;
                }
                break;

            case 5:
                robot.gyroTurn(180);
                break;

            case 6:
                robot.runToTarget(Movement.BACKWARD, 20, false);
                break;

            case 7:
                robot.encoderReset();
                break;

            case 8:
                if (left > right && left > middle) { //LEFT
                    robot.command = 9;
                } else if (right > left && right > middle) { //RIGHT
                    robot.command = 101;
                } else if (middle > right && middle > left) { //MIDDLE
                    robot.command = 1001;
                }
                break;

            case 9:
                robot.runToTarget(Movement.LEFTSTRAFE, 40, false);
                break;

            case 10:
                robot.encoderReset();
                break;

            case 11:
                robot.skystoneGrab();
                robot.encoderReset();
                break;

            case 12:
                robot.runToTarget(Movement.FORWARD, 30, false);
                break;

            case 13:
                robot.encoderReset();
                break;

            case 14:
                robot.runToTarget(Movement.LEFTSTRAFE, 140, false);
                break;

            case 15:
                robot.skystoneRelease();
                robot.encoderReset();
                break;

            case 16:
                robot.runToTarget(Movement.RIGHTSTRAFE, 20, false);
                break;

            case 17:
                robot.encoderReset();
                break;

            case 101:
                robot.runToTarget(Movement.LEFTSTRAFE, 20, false);
                break;

            case 102:
                robot.encoderReset();
                break;

            case 103:
                robot.skystoneGrab();
                robot.encoderReset();
                break;

            case 104:
                robot.runToTarget(Movement.FORWARD, 30, false);
                break;

            case 105:
                robot.encoderReset();
                break;

            case 106:
                robot.runToTarget(Movement.LEFTSTRAFE, 160, false);
                break;

            case 107:
                robot.skystoneRelease();
                robot.encoderReset();
                break;

            case 108:
                robot.runToTarget(Movement.RIGHTSTRAFE, 20, false);
                break;

            case 109:
                robot.encoderReset();
                break;

            case 1001:
                robot.runToTarget(Movement.LEFTSTRAFE, 2, false);
                break;

            case 1002:
                robot.encoderReset();
                break;

            case 1003:
                robot.skystoneGrab();
                robot.encoderReset();
                break;

            case 1004:
                robot.runToTarget(Movement.FORWARD, 30, false);
                break;

            case 1005:
                robot.encoderReset();
                break;

            case 1006:
                robot.runToTarget(Movement.LEFTSTRAFE, 180, false);
                break;

            case 1007:
                robot.skystoneRelease();
                robot.encoderReset();
                break;

            case 1008:
                robot.runToTarget(Movement.RIGHTSTRAFE, 20, false);
                break;

            case 1009:
                robot.encoderReset();
                break;
        }
        telemetry.addData("Case:", robot.command);
        telemetry.addData("right", right);
        telemetry.addData("middle", middle);
        telemetry.addData("left", left);
        telemetry.addData("block", block);
        telemetry.update();
    }
}

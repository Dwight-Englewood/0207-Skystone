package org.firstinspires.ftc.teamcode.Autonomous.NewActiveAuton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Autonomous.Methods.*;

import org.firstinspires.ftc.teamcode.Hardware.Movement;

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
                robot.runToTarget(Movement.RIGHTSTRAFE, 10, false);
                break;

            case 3:
                runtime.reset();
                robot.encoderReset();
                break;

            case 4:
                SkystoneDetect.Spot returnedloc = detector.getSkystonePosRed(telemetry);
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
                if (left > right && left > middle) {
                    block = 3; //LEFT
                    robot.gyroTurn(90);
                } else if (right > left && right > middle) {
                    block = 1; //RIGHT
                    robot.gyroTurn(90);
                } else if (middle > right && middle > left) {
                    block = 2; //MIDDLE
                    robot.gyroTurn(90);
                }
                break;

            case 6:
                robot.encoderReset();
                break;

            case 7:
                runtime.reset();
                robot.intakeL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.intakeR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.runToTarget(Movement.FORWARD, 65, false);
                break;

            case 8:
                robot.intakeAuton(2000, 0.8);
                if (runtime.milliseconds() > 2000) {
                    robot.command++;
                }
                break;

            case 9:
                robot.runToTarget(Movement.BACKWARD, 20, false);
                break;

            case 10:
                robot.encoderReset();
                break;

            case 11:
                if (block == 1) {
                    robot.command = 12;
                } else if (block == 2) {
                    robot.command = 140;
                } else {
                    robot.command = 1400;
                }
                break;

            case 12:
                runtime.reset();
                robot.intakeL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.intakeR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.runToTarget(Movement.LEFTSTRAFE, 152, true);
                break;

            case 13:
                robot.intakeAuton(2000, -0.8);
                if (runtime.milliseconds() > 2000) {
                    robot.command++;
                }
                break;

            case 14:
                robot.runToTarget(Movement.RIGHTSTRAFE, 132, true);
                break;
/*
            case 15:
                robot.encoderReset();
                break;

            case 140:
                runtime.reset();
                robot.runToTarget(Movement.LEFTSTRAFE, 132, true);
                break;

            case 141:
                robot.openClampAuton();
                break;

            case 142:
                runtime.reset();
                robot.runToTarget(Movement.RIGHTSTRAFE, 72, true);
                break;

            case 143:
                robot.closeClampAuton();
                break;

            case 144:
                runtime.reset();
                robot.runToTarget(Movement.LEFTSTRAFE, 92, true);
                break;

            case 145:
                robot.openClampAuton();
                break;

            case 146:
                robot.runToTarget(Movement.RIGHTSTRAFE, 20, true);
                break;

            case 147:
                robot.encoderReset();
                break;

            case 1400:
                runtime.reset();
                robot.runToTarget(Movement.LEFTSTRAFE, 112, true);
                break;

            case 1401:
                robot.openClampAuton();
                break;

            case 1402:
                runtime.reset();
                robot.runToTarget(Movement.RIGHTSTRAFE, 52, true);
                break;

            case 1403:
                robot.closeClampAuton();
                break;

            case 1404:
                runtime.reset();
                robot.runToTarget(Movement.LEFTSTRAFE, 72, true);
                break;

            case 1405:
                robot.openClampAuton();
                break;

            case 1406:
                robot.runToTarget(Movement.RIGHTSTRAFE, 10, true);
                break;

 */

            case 15:
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

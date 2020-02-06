package org.firstinspires.ftc.teamcode.Autonomous.NewActiveAuton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Autonomous.Methods.NewAutonMethods;
import org.firstinspires.ftc.teamcode.Autonomous.Methods.SkystoneDetect;
import org.firstinspires.ftc.teamcode.Hardware.Movement;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

@Autonomous(name = "BBox", group = "Autonomous")
public class BBox extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    NewAutonMethods robot = new NewAutonMethods();
    SkystoneDetect detector = new SkystoneDetect();

    int block;

    public int left = 0;
    public int right = 0;
    public int middle = 0;
    public int notvis = 0;

    public int blockBrick;

    public void init() {
        robot.init(hardwareMap, telemetry);
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

        runtime.reset();
/*
        SkystoneDetect.Spot returnedloc = detector.getSkystonePos(telemetry, blue);
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
        if (runtime.milliseconds() > 1500) {
            if (left > 1000) { //LEFT
                blockBrick = leftBrick;
            } else if (right > 1000) { //RIGHT
                blockBrick = rightBrick;
            } else if (middle > 1000) { //MIDDLE
                blockBrick = middleBrick;
            }
        }

 */
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
        detector.init(hardwareMap);
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

                if (runtime.milliseconds() > 2500) {
                    if (left > right && left > middle) { //LEFT
                        blockBrick = 1;
                        robot.encoderReset();
                    } else if (right > middle && right > left) { //RIGHT
                        blockBrick = 2;
                        robot.encoderReset();
                    } else if (middle > left && middle > right) { //MIDDLE
                        blockBrick = 3;
                        robot.encoderReset();
                    }
                }

            case 1:
                if (blockBrick == 1) { //LEFT
                    robot.command= 4;
                } else if (blockBrick == 2) { //RIGHT
                    robot.command = 103;
                } else if (blockBrick == 3) { //MIDDLE
                    robot.command = 1001;
                }
                break;

            case 4:
                runtime.reset();
                robot.runWithIntake(Movement.UPLEFT, 20*10,.8);
                break;

            case 5:
                if (runtime.milliseconds() > 1000) {
                    robot.encoderReset();
                }
                break;

            case 6:
                robot.runToTarget(Movement.BACKWARD, 30*1.5);
                break;

            case 7:
                robot.encoderReset();
                break;

            case 8:
                robot.gyroTurn(-90);
                break;

            case 9:
                robot.encoderReset();
                break;

            case 10:
                robot.runToTarget(Movement.FORWARD, 100);
                break;

            case 11:
                robot.encoderReset();
                break;

            case 12:
                robot.intakeL.setPower(-.8);
                robot.intakeR.setPower(-.8);
                robot.gyroTurn(-90);
                break;

            case 14:
                robot.intakeL.setPower(0);
                robot.intakeR.setPower(0);
                if (runtime.milliseconds() > 1000) {
                    robot.encoderReset();
                }
                break;

            case 15:
                robot.gyroTurn(188);
                break;

            case 16:
                robot.encoderReset();
                break;

            case 17:
                robot.runToTarget(Movement.FORWARD, 100);
                break;

            case 18:
                robot.encoderReset();
                break;

            case 103:
                robot.runToTarget(Movement.FORWARD, 20*5.2);
                break;

            case 104:
                robot.intakeL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.intakeR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.encoderReset();
                break;

            case 105:
                robot.intakeAuton(500, 1);
                break;

            case 106:
                robot.runToTarget(Movement.BACKWARD, 30);
                break;

            case 107:
                robot.encoderReset();
                break;

            case 108:
                robot.gyroTurn(-88);
                break;

            case 109:
                robot.encoderReset();
                break;

            case 110:
                robot.runToTarget(Movement.BACKWARD, 200);
                break;

            case 111:
                robot.encoderReset();
                break;

            case 1001:
                robot.runToTarget(Movement.RIGHTSTRAFE, 20);
                break;

            case 1002:
                robot.encoderReset();
                break;

            case 1003:
                robot.runToTarget(Movement.FORWARD, 20*5.2);
                break;

            case 1004:
                robot.intakeL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.intakeR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.encoderReset();
                break;

            case 1005:
                robot.intakeAuton(500, 1);
                break;

            case 1006:
                robot.runToTarget(Movement.BACKWARD, 30);
                break;

            case 1007:
                robot.encoderReset();
                break;

            case 1008:
                robot.gyroTurn(-88);
                break;

            case 1009:
                robot.encoderReset();
                break;

            case 1010:
                robot.runToTarget(Movement.BACKWARD, 200);
                break;

            case 1011:
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

package org.firstinspires.ftc.teamcode.Autonomous.NewActiveAuton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.Autonomous.Methods.NewAutonMethods;
import org.firstinspires.ftc.teamcode.Autonomous.Methods.SkystoneDetect;
import org.firstinspires.ftc.teamcode.Hardware.Movement;

@Autonomous(name = "BBox", group = "Autonomous")
public class BBox extends OpMode {
    NewAutonMethods robot = new NewAutonMethods();
    SkystoneDetect detector = new SkystoneDetect();

    int block;

    public void init() {
        robot.init(hardwareMap, telemetry); // init all ur motors and crap (NOTE: DO NOT INIT GYRO OR VISION IN THIS METHOD)

        new Thread()  {
            public void run() {
                robot.initGyro();
                robot.isGyroInit();// whatever ur init gyro method is on robot
            }
        }.start();

        new Thread(){
            public void run() {
                detector.init(hardwareMap, telemetry);
                detector.isInit();// whatever ur vision init method is
            }
        }.start();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        detector.detectionLoopBlue();
        if (robot.runtime.milliseconds() >= 5000){
            detector.resetValues();
            robot.runtime.reset();
            telemetry.addData("Left:", detector.left);
            telemetry.addData("Right:", detector.right);
            telemetry.addData("Middle:", detector.middle);
            telemetry.addData("Invisible:", detector.notvis);
            telemetry.update();
        }
        telemetry.addData("Left:", detector.left);
        telemetry.addData("Right:", detector.right);
        telemetry.addData("Middle:", detector.middle);
        telemetry.addData("Invisible:", detector.notvis);
        telemetry.update();
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        detector.blockFinder();
        robot.runtimeReset();
    }
    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        switch (robot.command) {
            case 0:
                robot.encoderReset();
                break;

            case 1:
                if (detector.leftBlock) { //LEFT
                    robot.command = 103;
                } else if (detector.rightBlock) { //RIGHT
                    robot.command = 4;
                } else if (detector.middleBlock) { //MIDDLE
                    robot.command = 1003;
                }
                break;

            case 4:
                robot.runWithIntake(Movement.FORWARD, 20*6,.8);
                break;

            case 5:
                if (robot.runtime.milliseconds() > 1000) {
                    robot.encoderReset();
                }
                break;

            case 6:
                robot.runToTarget(Movement.BACKWARD, 30*1.5);
                break;

            case 8:
                robot.gyroTurn(90);
                break;

            case 10:
                robot.runToTarget(Movement.FORWARD, 100);
                break;

            case 12:
                robot.intakeL.setPower(-.8);
                robot.intakeR.setPower(-.8);
                robot.gyroTurn(88);
                break;

            case 14:
                robot.intakeL.setPower(0);
                robot.intakeR.setPower(0);
                if (robot.runtime.milliseconds() > 1000) {
                    robot.encoderReset();
                }
                break;

            case 15:
                robot.gyroTurn(188);
                break;

            case 17:
                robot.runToTarget(Movement.FORWARD, 100);
                break;

            case 101:
                robot.runToTarget(Movement.RIGHTSTRAFE, 12);
                break;

            case 103:
                robot.runToTarget(Movement.FORWARD, 20);
                break;

            case 105:
                robot.gyroTurn(90);
                break;

            case 107:
                robot.runToTarget(Movement.RIGHTSTRAFE, 125);
                break;

            case 109:
                robot.runWithIntake(Movement.FORWARD, 12,.4);
                break;

            case 110:
                robot.intakeL.setPower(0);
                robot.intakeR.setPower(0);
                robot.encoderReset();
                break;

            case 111:
                robot.runToTarget(Movement.LEFTSTRAFE, 50);
                break;

            case 113:
                robot.runToTarget(Movement.BACKWARD, 150);
                break;

            case 115:
                robot.gyroTurn(0);
                break;

            case 117:
                robot.runWithIntake(Movement.LEFTSTRAFE, 10, -1);
                break;

            case 118:
                robot.intakeL.setPower(0);
                robot.intakeR.setPower(0);
                robot.encoderReset();
                break;

            case 119:
                robot.runToTarget(Movement.LEFTSTRAFE,225);
                break;

            case 121:
                robot.gyroTurn(90);
                break;

            case 123:
                robot.runToTarget(Movement.RIGHTSTRAFE, 100);
                break;

            case 125:
                robot.runWithIntake(Movement.FORWARD, 20,.4);
                break;

            case 126:
                robot.encoderReset();
                robot.intakeL.setPower(0);
                robot.intakeR.setPower(0);
                break;

            case 127:
                robot.runToTarget(Movement.LEFTSTRAFE,60);
                break;

            case 129:
                robot.runToTarget(Movement.BACKWARD, 220);
                break;

            case 131:
                robot.gyroTurn(0);
                break;

            case 133:
                robot.runWithIntake(Movement.LEFTSTRAFE, 40, -1);
                break;

            case 134:
                robot.intakeL.setPower(0);
                robot.intakeR.setPower(0);
                robot.encoderReset();
                break;

            case 1003:
                robot.runToTarget(Movement.FORWARD, 15);
                break;

            case 1005:
                robot.gyroTurn(90);
                break;

            case 1007:
                robot.runToTarget(Movement.RIGHTSTRAFE, 125);
                break;

            case 1009:
                robot.runWithIntake(Movement.FORWARD, 18,.4);
                break;

            case 1010:
                robot.intakeL.setPower(0);
                robot.intakeR.setPower(0);
                robot.encoderReset();
                break;

            case 1011:
                robot.runToTarget(Movement.LEFTSTRAFE, 50);
                break;

            case 1013:
                robot.runToTarget(Movement.BACKWARD, 150);
                break;

            case 1015:
                robot.gyroTurn(0);
                break;

            case 1017:
                robot.runWithIntake(Movement.LEFTSTRAFE, 10, -1);
                break;

            case 1018:
                robot.intakeL.setPower(0);
                robot.intakeR.setPower(0);
                robot.encoderReset();
                break;

            case 1019:
                robot.runToTarget(Movement.LEFTSTRAFE,230);
                break;

            case 1021:
                robot.gyroTurn(90);
                break;

            case 1023:
                robot.runToTarget(Movement.RIGHTSTRAFE, 80);
                break;

            case 1025:
                robot.runWithIntake(Movement.FORWARD, 20,.4);
                break;

            case 1026:
                robot.encoderReset();
                robot.intakeL.setPower(0);
                robot.intakeR.setPower(0);
                break;

            case 1027:
                robot.runToTarget(Movement.LEFTSTRAFE,50);
                break;

            case 1029:
                robot.runToTarget(Movement.BACKWARD, 220);
                break;

            case 1031:
                robot.gyroTurn(0);
                break;

            case 1033:
                robot.runWithIntake(Movement.LEFTSTRAFE, 40, -1);
                break;

            case 1034:
                robot.intakeL.setPower(0);
                robot.intakeR.setPower(0);
                robot.encoderReset();
                break;
        }
        telemetry.addData("Case:", robot.command);
        telemetry.addData("right", detector.right);
        telemetry.addData("middle", detector.middle);
        telemetry.addData("left", detector.left);
        telemetry.addData("block", block);
        telemetry.update();
    }
}

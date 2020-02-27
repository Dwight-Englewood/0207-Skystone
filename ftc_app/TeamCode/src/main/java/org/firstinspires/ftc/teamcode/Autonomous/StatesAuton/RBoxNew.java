package org.firstinspires.ftc.teamcode.Autonomous.StatesAuton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Autonomous.Methods.NewAutonMethods;
import org.firstinspires.ftc.teamcode.Autonomous.Methods.SkystoneDetect;
import org.firstinspires.ftc.teamcode.Hardware.Movement;

@Autonomous(name = "RBox New", group = "Autonomous")
public class RBoxNew extends OpMode {
    NewAutonMethods robot = new NewAutonMethods();
    SkystoneDetect detector = new SkystoneDetect();

    public void init() {
        robot.initNew(hardwareMap, telemetry); // init all ur motors and crap (NOTE: DO NOT INIT GYRO OR VISION IN THIS METHOD)

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
        //        detector.start();
            }
        }.start();
        robot.runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        detector.detectionLoopRed();
        if (robot.runtime.milliseconds() >= 5000){
            detector.resetValues();
            robot.runtime.reset();
        }
        telemetry.addData("rightConf", detector.rightConf);
        telemetry.addData("midConf", detector.midConf);
        telemetry.addData("leftConf", detector.leftConf);
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        detector.blockFinder();
        robot.runtime.reset();
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
                    robot.command = 4;
                } else if (detector.rightBlock) { //RIGHT
                    robot.command = 101;
                } else if (detector.middleBlock) { //MIDDLE
                    robot.command = 1001;
                }
                break;
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            case 4:
                robot.runToTarget(Movement.RIGHTSTRAFE, 25);
                break;

            case 5:
                robot.runToTarget(Movement.FORWARD, 17);
                break;

            case 6:
                robot.gyroTurn(90);
                break;

            case 7:
                robot.runToTarget(Movement.RIGHTSTRAFE, 105);
                break;

            case 8:
                robot.runWithIntake(Movement.FORWARD, 17,.7);
                break;

            case 9:
                robot.intakeL.setPower(0);
                robot.intakeR.setPower(0);
                robot.encoderReset();
                break;

            case 10:
                robot.runToTarget(Movement.LEFTSTRAFE, 70);
                break;

            case 11:
                robot.runToTarget(Movement.FORWARD, 170);
                break;

            case 12:
                robot.gyroTurn(0);
                break;

            case 13:
                robot.runWithIntake(Movement.RIGHTSTRAFE, 10, -1);
                break;

            case 14:
                robot.intakeL.setPower(0);
                robot.intakeR.setPower(0);
                robot.encoderReset();
                break;

            case 15:
                robot.runToTarget(Movement.RIGHTSTRAFE,205);
                break;

            case 16:
                robot.runWithIntake(Movement.FORWARD, 30,.7);
                break;

            case 17:
                robot.encoderReset();
                robot.intakeL.setPower(0);
                robot.intakeR.setPower(0);
                break;

            case 18:
                robot.gyroTurn(-90);
                break;

            case 19:
                robot.runToTarget(Movement.RIGHTSTRAFE,60);
                break;

            case 20:
                robot.runToTarget(Movement.BACKWARD, 170);
                break;

            case 21:
                robot.gyroTurn(0);
                break;

            case 22:
                robot.runWithIntake(Movement.RIGHTSTRAFE, 60, -1);
                break;

            case 23:
                robot.intakeL.setPower(0);
                robot.intakeR.setPower(0);
                robot.encoderReset();
                break;
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            case 101:
                robot.runToTarget(Movement.LEFTSTRAFE, 16.5);
                break;

            case 102:
                robot.runToTarget(Movement.FORWARD, 15);
                break;

            case 103:
                robot.gyroTurn(-90);
                break;

            case 104:
                robot.runToTarget(Movement.LEFTSTRAFE, 105);
                break;

            case 105:
                robot.runWithIntake(Movement.FORWARD, 17,.7);
                break;

            case 106:
                robot.intakeL.setPower(0);
                robot.intakeR.setPower(0);
                robot.encoderReset();
                break;

            case 107:
                robot.runToTarget(Movement.RIGHTSTRAFE, 75);
                break;

            case 108:
                robot.runToTarget(Movement.BACKWARD, 150);
                break;

            case 109:
                robot.gyroTurn(0);
                break;

            case 110:
                robot.runWithIntake(Movement.RIGHTSTRAFE, 10, -1);
                break;

            case 111:
                robot.intakeL.setPower(0);
                robot.intakeR.setPower(0);
                robot.encoderReset();
                break;

            case 112:
                robot.runToTarget(Movement.RIGHTSTRAFE,215);
                break;

            case 113:
                robot.gyroTurn(-90);
                break;

            case 114:
                robot.runToTarget(Movement.LEFTSTRAFE, 80);
                break;

            case 115:
                robot.runWithIntake(Movement.FORWARD, 17,7);
                break;

            case 116:
                robot.encoderReset();
                robot.intakeL.setPower(0);
                robot.intakeR.setPower(0);
                break;

            case 117:
                robot.runToTarget(Movement.RIGHTSTRAFE,50);
                break;

            case 118:
                robot.runToTarget(Movement.BACKWARD, 230);
                break;

            case 119:
                robot.gyroTurn(0);
                break;

            case 120:
                robot.runWithIntake(Movement.RIGHTSTRAFE, 60, -1);
                break;

            case 121:
                robot.intakeL.setPower(0);
                robot.intakeR.setPower(0);
                robot.encoderReset();
                break;
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            case 1001:
                robot.runToTarget(Movement.LEFTSTRAFE, 40);
                break;

            case 1002:
                robot.runToTarget(Movement.FORWARD, 15);
                break;

            case 1003:
                robot.gyroTurn(-90);
                break;

            case 1004:
                robot.runToTarget(Movement.LEFTSTRAFE, 105);
                break;

            case 1005:
                robot.runWithIntake(Movement.FORWARD, 17,.7);
                break;

            case 1006:
                robot.intakeL.setPower(0);
                robot.intakeR.setPower(0);
                robot.encoderReset();
                break;

            case 1007:
                robot.runToTarget(Movement.RIGHTSTRAFE, 65);
                break;

            case 1008:
                robot.runToTarget(Movement.BACKWARD, 150);
                break;

            case 1009:
                robot.gyroTurn(0);
                break;

            case 1010:
                robot.runWithIntake(Movement.RIGHTSTRAFE, 10, -1);
                break;

            case 1011:
                robot.intakeL.setPower(0);
                robot.intakeR.setPower(0);
                robot.encoderReset();
                break;

            case 1012:
                robot.runToTarget(Movement.RIGHTSTRAFE,205);
                break;

            case 1013:
                robot.gyroTurn(-90);
                break;

            case 1014:
                robot.runToTarget(Movement.LEFTSTRAFE, 72.5);
                break;

            case 1015:
                robot.runWithIntake(Movement.FORWARD, 17,.7);
                break;

            case 1016:
                robot.encoderReset();
                robot.intakeL.setPower(0);
                robot.intakeR.setPower(0);
                break;

            case 1017:
                robot.runToTarget(Movement.RIGHTSTRAFE,60);
                break;

            case 1018:
                robot.runToTarget(Movement.BACKWARD, 220);
                break;

            case 1019:
                robot.gyroTurn(0);
                break;

            case 1020:
                robot.runWithIntake(Movement.RIGHTSTRAFE, 70, -1);
                break;

            case 1021:
                robot.intakeL.setPower(0);
                robot.intakeR.setPower(0);
                robot.encoderReset();
                break;
        }
        telemetry.addData("Case:", robot.command);
        telemetry.addData("rightConf", detector.rightConf);
        telemetry.addData("midConf", detector.midConf);
        telemetry.addData("leftConf", detector.leftConf);
        telemetry.update();
    }
}

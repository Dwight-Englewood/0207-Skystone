package org.firstinspires.ftc.teamcode.Autonomous.NewActiveAuton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.Autonomous.Methods.NewAutonMethods;
import org.firstinspires.ftc.teamcode.Autonomous.Methods.SkystoneDetect;
import org.firstinspires.ftc.teamcode.Hardware.Movement;

@Autonomous(name = "RBox", group = "Autonomous")
public class RBox extends OpMode {
    NewAutonMethods robot = new NewAutonMethods();
    SkystoneDetect detector = new SkystoneDetect();

    int block;

    public int left = 0;
    public int right = 0;
    public int middle = 0;
    public int notvis = 0;

    public int blockBrick;

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
                detector.init(hardwareMap);
                detector.isInit();// whatever ur vision init method is
            }
        }.start();
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
        robot.runtime.reset();
    }
    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        switch (robot.command) {
            case 0:
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

                if (robot.runtime.milliseconds() > 2500) {
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
                    robot.command = 101;
                } else if (blockBrick == 3) { //MIDDLE
                    robot.command = 1001;
                }
                break;

            case 2:
                robot.runToTarget(Movement.LEFTSTRAFE, 20);
                break;

            case 3:
                robot.runtime.reset();
                robot.encoderReset();
                break;

            case 4:
                robot.runtime.reset();
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

            case 7:
                robot.encoderReset();
                break;

            case 8:
                robot.gyroTurn(90);
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

            case 16:
                robot.encoderReset();
                break;

            case 17:
                robot.runToTarget(Movement.FORWARD, 100);
                break;

            case 18:
                robot.encoderReset();
                break;

            case 101:
                robot.runToTarget(Movement.LEFTSTRAFE, 12);
                break;

            case 102:
                robot.encoderReset();
                break;

            case 103:
                robot.runToTarget(Movement.FORWARD, 2.5*5.2);
                break;

            case 104:
                robot.encoderReset();
                break;

            case 105:
                robot.gyroTurn(-90);
                break;

            case 106:
                robot.encoderReset();
                break;

            case 107:
                robot.runToTarget(Movement.LEFTSTRAFE, 125);
                break;

            case 108:
                robot.encoderReset();
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
                robot.runToTarget(Movement.RIGHTSTRAFE, 50);
                break;

            case 112:
                robot.encoderReset();
                break;

            case 113:
                robot.runToTarget(Movement.BACKWARD, 150);
                break;

            case 114:
                robot.encoderReset();
                break;

            case 115:
                robot.gyroTurn(0);
                break;

            case 116:
                robot.encoderReset();
                break;

            case 117:
                robot.runWithIntake(Movement.RIGHTSTRAFE, 10, -1);
                break;

            case 118:
                robot.intakeL.setPower(0);
                robot.intakeR.setPower(0);
                robot.encoderReset();
                break;

            case 119:
                robot.runToTarget(Movement.RIGHTSTRAFE,225);
                break;

            case 120:
                robot.encoderReset();
                break;

            case 121:
                robot.gyroTurn(-90);
                break;

            case 122:
                robot.encoderReset();
                break;

            case 123:
                robot.runToTarget(Movement.LEFTSTRAFE, 100);
                break;

            case 124:
                robot.encoderReset();
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
                robot.runToTarget(Movement.RIGHTSTRAFE,60);
                break;

            case 128:
                robot.encoderReset();
                break;

            case 129:
                robot.runToTarget(Movement.BACKWARD, 220);
                break;

            case 130:
                robot.encoderReset();
                break;

            case 131:
                robot.gyroTurn(0);
                break;

            case 132:
                robot.encoderReset();
                break;

            case 133:
                robot.runWithIntake(Movement.RIGHTSTRAFE, 40, -1);
                break;

            case 134:
                robot.intakeL.setPower(0);
                robot.intakeR.setPower(0);
                robot.encoderReset();
                break;

            case 1001:
                robot.runToTarget(Movement.LEFTSTRAFE, 40);
                break;

            case 1002:
                robot.encoderReset();
                break;

            case 1003:
                robot.runToTarget(Movement.FORWARD, 15);
                break;

            case 1004:
                robot.encoderReset();
                break;

            case 1005:
                robot.gyroTurn(-90);
                break;

            case 1006:
                robot.encoderReset();
                break;

            case 1007:
                robot.runToTarget(Movement.LEFTSTRAFE, 125);
                break;

            case 1008:
                robot.encoderReset();
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
                robot.runToTarget(Movement.RIGHTSTRAFE, 50);
                break;

            case 1012:
                robot.encoderReset();
                break;

            case 1013:
                robot.runToTarget(Movement.BACKWARD, 150);
                break;

            case 1014:
                robot.encoderReset();
                break;

            case 1015:
                robot.gyroTurn(0);
                break;

            case 1016:
                robot.encoderReset();
                break;

            case 1017:
                robot.runWithIntake(Movement.RIGHTSTRAFE, 10, -1);
                break;

            case 1018:
                robot.intakeL.setPower(0);
                robot.intakeR.setPower(0);
                robot.encoderReset();
                break;

            case 1019:
                robot.runToTarget(Movement.RIGHTSTRAFE,230);
                break;

            case 1020:
                robot.encoderReset();
                break;

            case 1021:
                robot.gyroTurn(-90);
                break;

            case 1022:
                robot.encoderReset();
                break;

            case 1023:
                robot.runToTarget(Movement.LEFTSTRAFE, 80);
                break;

            case 1024:
                robot.encoderReset();
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
                robot.runToTarget(Movement.RIGHTSTRAFE,50);
                break;

            case 1028:
                robot.encoderReset();
                break;

            case 1029:
                robot.runToTarget(Movement.BACKWARD, 220);
                break;

            case 1030:
                robot.encoderReset();
                break;

            case 1031:
                robot.gyroTurn(0);
                break;

            case 1032:
                robot.encoderReset();
                break;

            case 1033:
                robot.runWithIntake(Movement.RIGHTSTRAFE, 40, -1);
                break;

            case 1034:
                robot.intakeL.setPower(0);
                robot.intakeR.setPower(0);
                robot.encoderReset();
                break;



        }
        telemetry.addData("Case:", robot.command);
        telemetry.addData("right", right);
        telemetry.addData("middle", middle);
        telemetry.addData("left", left);
        telemetry.addData("block", block);
        telemetry.addData("intakeL current", robot.intakeL.getCurrentPosition());
        telemetry.addData("intakeL target", robot.intakeL.getTargetPosition());
        telemetry.update();
    }
}

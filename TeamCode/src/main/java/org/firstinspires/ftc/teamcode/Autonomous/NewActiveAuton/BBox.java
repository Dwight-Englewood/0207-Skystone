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
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
   //     detector.start();
        robot.runtimeReset();
    }
    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        switch (robot.command) {
            case 1:
                if (detector.leftBlock) { //LEFT
                    robot.command = 4;
                } else if (detector.rightBlock) { //RIGHT
                    robot.command = 103;
                } else if (detector.middleBlock) { //MIDDLE
                    robot.command = 1001;
                }
                break;

            case 4:
                robot.runtime.reset();
                robot.runWithIntake(Movement.UPLEFT, 20*10,.8);
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

            case 103:
                robot.runToTarget(Movement.FORWARD, 20*5.2);
                break;

            case 104:
                robot.intakeReset();
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
                robot.intakeReset();
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
        telemetry.addData("right", detector.right);
        telemetry.addData("middle", detector.middle);
        telemetry.addData("left", detector.left);
        telemetry.addData("block", block);
        telemetry.update();
    }
}

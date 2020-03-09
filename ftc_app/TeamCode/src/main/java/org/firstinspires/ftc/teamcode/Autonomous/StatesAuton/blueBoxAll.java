package org.firstinspires.ftc.teamcode.Autonomous.StatesAuton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Autonomous.Methods.NewAutonMethods;
import org.firstinspires.ftc.teamcode.Autonomous.Methods.SkystoneDetect;
import org.firstinspires.ftc.teamcode.Hardware.Movement;
@Disabled
@Autonomous(name = "BBOX All", group = "Autonomous")
public class blueBoxAll extends OpMode {
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
        detector.detectionLoopBlue();
        if (robot.runtime.milliseconds() >= 1000){
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
        robot.lowerRightClaws();
        robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        switch (robot.command) {
            case 0:
                if (detector.leftBlock) { //LEFT
                    robot.command = 4;
                } else if (detector.rightBlock) { //RIGHT
                    robot.command = 101;
                } else if (detector.middleBlock) { //MIDDLE
                    robot.command = 1001;
                }
                break;

            case 1:

                break;

            case 4:
                robot.setTarget(Movement.RIGHTSTRAFE, 20);
                break;

            case 1001:
                robot.setTarget(Movement.FORWARD, 20);
                break;

            case 1002:
                robot.finishDrive();
                break;

            case 1003:
                robot.gyroTurn(85);
                break;

            case 1004:
                robot.setTarget(Movement.FORWARD, 67);
                break;

            case 1005:
                robot.finishDrive();
                break;

            case 1006:
                robot.gyroTurn(90);
                break;

            case 1007:
                robot.setTarget(Movement.RIGHTSTRAFE, 68);
                break;

            case 1008:
                robot.finishDrive();
                break;

            case 1009:
                robot.closeRightClaws();
                if (robot.runtime.milliseconds() > 500) {
                    robot.command++;
                }
                break;

            case 1010:
                robot.liftRightClaws();
                robot.setTarget(Movement.LEFTSTRAFE, 37);
                break;

            case 1011:
                robot.finishDrive();
                break;

            case 1012:
                robot.gyroTurn(90);
                break;

            case 1013:
                robot.setTarget(Movement.BACKWARD, 180);
                break;

            case 1014:
                robot.finishDrive();
                break;

            case 1015:
                robot.lowerRightClaws();
                robot.gyroTurn(90);
                break;

            case 1016:
                robot.openRightClaws();
                if (robot.runtime.milliseconds() > 500) {
                    robot.command++;
                }
                break;

            case 1017:
                robot.setTarget(Movement.FORWARD, 110);
                break;

            case 1018:
                robot.finishDrive();
                break;

            case 1019:
                robot.setTarget(Movement.RIGHTSTRAFE, 45);
                break;

            case 1020:
                robot.lowerRightClaws();
                robot.finishDrive();
                break;

            case 1021:
                robot.closeRightClaws();
                if (robot.runtime.milliseconds() > 500) {
                    robot.command++;
                }
                break;

            case 1022:
                robot.liftRightClaws();
                robot.setTarget(Movement.LEFTSTRAFE, 37);
                break;

            case 1023:
                robot.finishDrive();
                break;

            case 1024:
                robot.gyroTurn(90);
                break;

            case 1025:
                robot.setTarget(Movement.BACKWARD, 120);
                break;

            case 1026:
                robot.finishDrive();
                break;

            case 1027:
                robot.lowerRightClaws();
                robot.gyroTurn(90);
                break;

            case 1028:
                robot.openRightClaws();
                if (robot.runtime.milliseconds() > 500) {
                    robot.command++;
                }
                break;

            case 1029:
                robot.setTarget(Movement.FORWARD, 100);
                break;

            case 1030:
                robot.finishDrive();
                break;

            case 1031:
                robot.setTarget(Movement.RIGHTSTRAFE, 45);
                break;

            case 1032:
                robot.lowerRightClaws();
                robot.finishDrive();
                break;

            case 1033:
                robot.closeRightClaws();
                if (robot.runtime.milliseconds() > 800) {
                    robot.command++;
                }
                break;

            case 1034:
                robot.liftRightClaws();
                robot.setTarget(Movement.LEFTSTRAFE, 37);
            break;

            case 1035:
                robot.finishDrive();
                break;

            case 1036:
                robot.gyroTurn(90);
                break;

         /*   case 1038:
                robot.setTarget(Movement.BACKWARD, 170);
                break;

          */

            case 1039:
                robot.finishDrive();
                break;

            case 1040:
                robot.setTarget(Movement.DOWNLEFT, 50);
                break;

            case 1041:
                robot.finishDrive();
                break;

            case 1042:
                robot.lowerRightClaws();
                robot.openRightClaws();
                robot.gyroTurn(90);
                break;
        }
        telemetry.addData("Case:", robot.command);
        telemetry.addData("Heading", robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
        telemetry.update();
    }
}

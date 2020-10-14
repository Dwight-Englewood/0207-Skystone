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

@Autonomous(name = "NewClawsBlueBox", group = "Autonomous")
public class NewClawsBlueBox extends OpMode {
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
        robot.initClaws();
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
        robot.lowerOpenLeftClaws();
        robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        switch (robot.command) {
            case 0:
                if (detector.leftBlock) {
                    robot.command = 1;
                } else if (detector.rightBlock) {
                    robot.command = 100;
                } else if (detector.middleBlock) {
                    robot.command = 1000;
                }
            case 100:
                robot.setTarget(Movement.LEFTSTRAFE, 53);
                break;

            case 101:
                robot.finishDrive();
                break;

            case 102:
                robot.setTarget(Movement.FORWARD, 82);
                break;

            case 103:
                robot.finishDrive();
                break;

            case 104:
                robot.closeLeftClaws();
                if (robot.runtime.milliseconds() > 500) {
                    robot.command++;
                }
                break;

            case 105:
                robot.setTarget(Movement.BACKWARD, 35);
                break;

            case 106:
                robot.liftLeftClaws();
                robot.finishDrive();
                break;

            case 107:
                robot.setTarget(Movement.RIGHTSTRAFE, 280);
                break;

            case 108:
                robot.finishDrive();
                break;

            case 109:
                robot.setTarget(Movement.FORWARD, 33);
                break;

            case 110:
                robot.finishDrive();
                break;

            case 111:
                robot.setTarget(Movement.BACKWARD, 30);
                break;

            case 112:
                robot.lowerOpenLeftClaws();
                robot.finishDrive();
                break;
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            case 113:
                robot.setTarget(Movement.LEFTSTRAFE, 220);
                break;

            case 114:
                robot.liftLeftClaws();
                robot.finishDrive();
                break;

            case 115:
                robot.setTarget(Movement.FORWARD, 35);
                break;

            case 116:
                robot.lowerOpenLeftClaws();
                robot.finishDrive();
                break;

            case 117:
                robot.closeLeftClaws();
                if (robot.runtime.milliseconds() > 500) {
                    robot.command++;
                }
                break;

            case 118:
                robot.setTarget(Movement.BACKWARD, 26);
                break;

            case 119:
                robot.liftLeftClaws();
                robot.finishDrive();
                break;

            case 120:
                robot.setTarget(Movement.RIGHTSTRAFE, 234);
                break;

            case 121:
                robot.finishDrive();
                break;

            case 122:
                robot.setTarget(Movement.FORWARD, 28);
                break;

            case 123:
                robot.finishDrive();
                break;

            case 124:
                robot.setTarget(Movement.BACKWARD, 31);
                break;

            case 125:
                robot.lowerOpenLeftClaws();
                robot.finishDrive();
                break;
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            case 126:
                robot.setTarget(Movement.LEFTSTRAFE, 259);
                break;

            case 127:
                robot.liftLeftClaws();
                robot.finishDrive();
                break;

            case 128:
                robot.setTarget(Movement.FORWARD, 44);
                break;

            case 129:
                robot.lowerOpenLeftClaws();
                robot.finishDrive();
                break;

            case 130:
                robot.closeLeftClaws();
                if (robot.runtime.milliseconds() > 500) {
                    robot.command++;
                }
                break;

            case 131:
                robot.setTarget(Movement.BACKWARD, 37);
                break;

            case 132:
                robot.liftLeftClaws();
                robot.finishDrive();
                break;

            case 133:
                robot.setTarget(Movement.RIGHTSTRAFE, 245);
                break;

            case 134:
                robot.finishDrive();
                break;

            case 135:
                robot.setTarget(Movement.FORWARD, 29);
                break;

            case 136:
                robot.finishDrive();
                break;

            case 137:
                robot.setTarget(Movement.BACKWARD, 35);
                break;

            case 138:
                robot.lowerOpenLeftClaws();
                robot.finishDrive();
                break;

            case 139:
                robot.gyroTurn(175);
                robot.openServoAuton();
                break;

            case 140:
                robot.setTarget(Movement.BACKWARD, 70);
                break;

            case 141:
                robot.liftLeftClaws();
                robot.finishDrive();
                break;

            case 142:
                robot.closeServoAuton();
                if (robot.runtime.milliseconds() > 500) {
                    robot.command++;
                }
                break;

            case 143:
                robot.setTarget(Movement.FORWARD, 110);
                break;

            case 144:
                robot.finishDrive();
                break;

            case 145:
                robot.setTarget(Movement.RIGHTSTRAFE, 130);
                break;

            case 146:
                robot.openServoAuton();
                robot.finishDrive();
                break;

        }
        telemetry.addData("Case:", robot.command);

        telemetry.addData("Failsafe:", robot.timerFailSafe());
        telemetry.addData("runtime", robot.runtime.milliseconds());

        telemetry.addData("Powersafe:", robot.tinyPowerValue());
        telemetry.addData("power", robot.FL.getPower());
        telemetry.update();
    }
}

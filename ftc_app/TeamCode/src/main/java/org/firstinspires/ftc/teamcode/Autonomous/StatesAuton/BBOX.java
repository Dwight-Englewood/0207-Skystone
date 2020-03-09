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

@Autonomous(name = "Bbox", group = "Autonomous")
public class BBOX extends OpMode {
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
                robot.setTarget(Movement.FORWARD, 20);
                break;

            case 1:
                robot.finishDrive();
                break;

            case 2:
                robot.gyroTurn(87);
                break;
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            case 3:
                robot.setTarget(Movement.FORWARD, 68);
                break;

            case 4:
                robot.finishDrive();
                break;

            case 5:
                robot.gyroTurn(85);
                break;

            case 6:
                robot.setTarget(Movement.RIGHTSTRAFE, 68);
                break;

            case 7:
                robot.finishDrive();
                break;

            case 8:
                robot.closeRightClaws();
                if (robot.runtime.milliseconds() > 500) {
                    robot.command++;
                }
                break;

            case 9:
                robot.liftRightClaws();
                robot.setTarget(Movement.LEFTSTRAFE, 30);
                break;

            case 10:
                robot.finishDrive();
                break;

            case 11:
                robot.gyroTurn(92);
                break;

            case 12:
                robot.setTarget(Movement.BACKWARD, 250);
                break;

            case 13:
                robot.finishDrive();
                break;

            case 14:
                robot.setTarget(Movement.RIGHTSTRAFE, 35);
                break;

            case 15:
                robot.finishDrive();
                break;

            case 16:
                robot.lowerRightClaws();
             //   robot.gyroTurn(90);
                robot.command++;
                break;

            case 17:
                robot.setTarget(Movement.LEFTSTRAFE, 33);
                break;

            case 18:
                robot.openRightClaws();
                robot.finishDrive();
                break;
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            case 19:
                robot.setTarget(Movement.FORWARD, 200);
                break;

            case 20:
                robot.liftRightClaws();
                robot.finishDrive();
                break;

            case 21:
             //   robot.gyroTurn(85);
                robot.command++;
                break;

            case 22:
                robot.setTarget(Movement.RIGHTSTRAFE, 35);
                break;

            case 23:
                robot.lowerRightClaws();
                robot.finishDrive();
                break;

            case 24:
                robot.closeRightClaws();
           //     robot.gyroTurn(85);
                robot.command++;
                break;

            case 25:
                robot.liftRightClaws();
                robot.setTarget(Movement.LEFTSTRAFE, 30);
                break;

            case 26:
                robot.finishDrive();
                break;

            case 27:
         //       robot.gyroTurn(92);
                robot.command++;
                break;

            case 28:
                robot.setTarget(Movement.BACKWARD, 170);
                break;

            case 29:
                robot.finishDrive();
                break;

            case 30:
                robot.setTarget(Movement.RIGHTSTRAFE, 32);
                break;

            case 31:
                robot.finishDrive();
                break;

            case 32:
                robot.lowerRightClaws();
             //   robot.gyroTurn(90);
                robot.command++;
                break;

            case 33:
                robot.setTarget(Movement.LEFTSTRAFE, 33);
                break;

            case 34:
                robot.openRightClaws();
                robot.finishDrive();
                break;
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            case 35:
                robot.setTarget(Movement.FORWARD, 160);
                break;

            case 36:
                robot.finishDrive();
                break;

            case 37:
           //     robot.gyroTurn(85);
                robot.command++;
                break;

            case 38:
                robot.setTarget(Movement.RIGHTSTRAFE, 45);
                break;

            case 39:
                robot.lowerRightClaws();
                robot.finishDrive();
                break;

            case 40:
                robot.closeRightClaws();
                if (robot.runtime.milliseconds() > 800) {
                    robot.command++;
                }
                break;

            case 41:
                robot.liftRightClaws();
                robot.setTarget(Movement.LEFTSTRAFE, 33);
                break;

            case 42:
                robot.finishDrive();
                break;

            case 43:
            //    robot.gyroTurn(92);
                robot.command++;
                break;

            case 44:
                robot.setTarget(Movement.BACKWARD, 160);
                break;

            case 45:
                robot.finishDrive();
                break;

            case 46:
                robot.setTarget(Movement.RIGHTSTRAFE, 45);
                break;

            case 47:
                robot.finishDrive();
                break;

            case 48:
                robot.lowerRightClaws();
             //   robot.gyroTurn(90);
                robot.command++;
                break;

            case 49:
                robot.setTarget(Movement.LEFTSTRAFE, 35);
                break;

            case 50:
                robot.openRightClaws();
                robot.finishDrive();
                break;
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            case 51:
                robot.setTarget(Movement.FORWARD, 160);
                break;

            case 52:
                robot.finishDrive();
                break;
        }
        telemetry.addData("Case:", robot.command);
        telemetry.addData("Current Heading", robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);

        telemetry.addData("FL Current", robot.FL.getCurrentPosition());
        telemetry.addData("FL Target", robot.FL.getTargetPosition());
        telemetry.update();
    }
}

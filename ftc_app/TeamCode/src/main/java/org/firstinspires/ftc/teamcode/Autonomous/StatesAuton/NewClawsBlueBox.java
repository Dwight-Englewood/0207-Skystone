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

@Autonomous(name = "NewCLawsBlueBox", group = "Autonomous")
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
                robot.setTarget(Movement.FORWARD, 68);
                break;

            case 1:
                robot.finishDrive();
                break;

            case 2:
                robot.closeRightClaws();
                if (robot.runtime.milliseconds() > 500) {
                    robot.command++;
                }
                break;

            case 3:
                robot.setTarget(Movement.BACKWARD, 30);
                break;

            case 4:
                robot.finishDrive();
                break;

            case 5:
                robot.liftRightClaws();
                robot.setTarget(Movement.RIGHTSTRAFE, 220);
                break;

            case 6:
                robot.finishDrive();
                break;

            case 7:
                robot.setTarget(Movement.FORWARD, 40);
                break;

            case 8:
                robot.lowerRightClaws();
                robot.openRightClaws();
                robot.finishDrive();
                break;
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            case 9:
                robot.setTarget(Movement.BACKWARD, 35);
                break;

            case 10:
                robot.liftRightClaws();
                robot.finishDrive();
                break;

            case 11:
                robot.setTarget(Movement.LEFTSTRAFE, 280);
                break;

            case 12:
                robot.finishDrive();
                break;

            case 13:
                robot.setTarget(Movement.FORWARD, 45);
                break;

            case 14:
                robot.lowerRightClaws();
                robot.finishDrive();
                break;

            case 15:
                robot.setTarget(Movement.BACKWARD, 35);
                break;

            case 16:
                robot.closeRightClaws();
                robot.finishDrive();
                break;

            case 17:
                robot.setTarget(Movement.RIGHTSTRAFE, 280);
                break;

            case 18:
                robot.liftRightClaws();
                robot.finishDrive();
                break;

            case 19:
                robot.setTarget(Movement.FORWARD, 50);
                break;

            case 20:
                robot.lowerRightClaws();
                robot.openRightClaws();
                robot.finishDrive();
                break;

            case 21:
                robot.setTarget(Movement.BACKWARD, 35);
                break;

            case 22:
                robot.liftRightClaws();
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

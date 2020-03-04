package org.firstinspires.ftc.teamcode.Autonomous.NewActiveAuton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Autonomous.Methods.NewAutonMethods;
import org.firstinspires.ftc.teamcode.Hardware.Movement;

@Autonomous(name = "BFound", group = "Autonomous")
public class BlueFound extends OpMode {
    NewAutonMethods robot = new NewAutonMethods();

    public void init() {
        robot.init(hardwareMap, telemetry); // init all ur motors and crap (NOTE: DO NOT INIT GYRO OR VISION IN THIS METHOD)

        new Thread()  {
            public void run() {
                robot.initGyro();
                robot.isGyroInit();// whatever ur init gyro method is on robot
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
        robot.runtime.reset();
        robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        switch (robot.command) {
            case 0:
                robot.setTarget(Movement.DOWNLEFT, 190);
                break;

            case 1:
                robot.finishDrive();
                break;

            case 2:
                robot.newCloseServoAuton();
                if (robot.runtime.milliseconds() > 500) {
                    robot.command++;
                }
                break;

            case 3:
                robot.setTarget(Movement.UPRIGHT, 125);
                break;

            case 4:
                robot.finishDrive();
                break;

            case 5:
                robot.gyroTurn(89);
                break;

            case 6:
                robot.setTarget(Movement.DOWNLEFT , 140);
                break;

            case 7:
                robot.finishDrive();
                break;

            case 8:
                robot.gyroTurn(89);
                break;

            case 9:
                robot.newOpenServoAuton();
                if (robot.runtime.milliseconds() > 500) {
                    robot.command++;
                }
                break;

            case 10:
                robot.setTarget(Movement.BACKWARD,15);
                break;

            case 11:
                robot.finishDrive();
                break;

            case 12:
                robot.setTarget(Movement.RIGHTSTRAFE,132);
                break;

            case 13:
                robot.finishDrive();
                break;

            case 14:
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                break;
        }
        telemetry.addData("Case:", robot.command);
        telemetry.addData("Current Heading", robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
        telemetry.addData("FL Current", robot.FL.getCurrentPosition());
        telemetry.addData("FL Target", robot.FL.getTargetPosition());
        telemetry.addData("FL Power", robot.FL.getPower());
        telemetry.update();
    }
}


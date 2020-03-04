package org.firstinspires.ftc.teamcode.Autonomous.NewActiveAuton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.Autonomous.Methods.NewAutonMethods;
import org.firstinspires.ftc.teamcode.Hardware.Movement;

@Autonomous(name = "TestMet", group = "Autonomous")
public class TestMet extends OpMode {
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
                robot.runNow(Movement.FORWARD, 70, 0.5);
                break;

            case 1:
                robot.finishDrive();
                break;

            case 2:
                robot.runNow(Movement.BACKWARD, 30, 0.5);
                break;

            case 3:
                robot.finishDrive();
                break;

            case 4:
                break;
        }
        telemetry.addData("Case:", robot.command);
        telemetry.addData("FL Target", robot.FL.getTargetPosition());
        telemetry.addData("FL Current", robot.FL.getCurrentPosition());
        telemetry.addData("FL Power", robot.FL.getPower());

        telemetry.addData("FR Target", robot.FR.getTargetPosition());
        telemetry.addData("FR Current", robot.FR.getCurrentPosition());
        telemetry.addData("FR Power", robot.FR.getPower());

        telemetry.addData("BL Target", robot.BL.getTargetPosition());
        telemetry.addData("BL Current", robot.BL.getCurrentPosition());
        telemetry.addData("BL Power", robot.BL.getPower());

        telemetry.addData("BR Target", robot.BR.getTargetPosition());
        telemetry.addData("BR Current", robot.BR.getCurrentPosition());
        telemetry.addData("BR Power", robot.BR.getPower());

        telemetry.addData("P", robot.error * robot.kpVal);
        telemetry.addData("I", robot.errorI * robot.kiVal);
        telemetry.addData("D", robot.errorD * robot.kdVal);
        telemetry.update();
    }
}


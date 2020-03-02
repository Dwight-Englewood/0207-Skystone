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
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        switch (robot.command) {
            case 0:
                robot.setTarget(Movement.FORWARD, 50);
                break;

            case 1:
                robot.finishDrive();
                break;

            case 2:
                robot.setTarget(Movement.FORWARD, 100);
                break;

            case 3:
                robot.finishDrive();
                break;

            case 4:
                robot.setTarget(Movement.BACKWARD, 150);
                break;

            case 5:
                robot.gyroTurn(0);
                break;

        }
        telemetry.addData("Case:", robot.command);
        telemetry.addData("FL Current", robot.FL.getCurrentPosition());
        telemetry.addData("FL Target", robot.FL.getTargetPosition());
        telemetry.addData("FL Power", robot.FL.getPower());

        telemetry.addData("P", robot.error * robot.kpVal);
        telemetry.addData("I", robot.errorI * robot.kiVal);
        telemetry.addData("D", robot.errorD * robot.kdVal);
        telemetry.update();
    }
}


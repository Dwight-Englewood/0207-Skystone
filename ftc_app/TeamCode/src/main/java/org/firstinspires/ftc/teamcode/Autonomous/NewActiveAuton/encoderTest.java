package org.firstinspires.ftc.teamcode.Autonomous.NewActiveAuton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.Autonomous.Methods.NewAutonMethods;
import org.firstinspires.ftc.teamcode.Hardware.Movement;

@Autonomous(name = "encoderTest", group = "Autonomous")
public class encoderTest extends OpMode {
    NewAutonMethods robot = new NewAutonMethods();

    public void init() {
        robot.init(hardwareMap, telemetry); // init all ur motors and crap (NOTE: DO NOT INIT GYRO OR VISION IN THIS METHOD)

        new Thread() {
            public void run() {
                robot.initGyro();
                robot.isGyroInit();// whatever ur init gyro method is on robot
            }
        }.start();    }

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
                robot.encoderRun(Movement.FORWARD, 80);
                break;

            case 1:
                robot.encoderRun(Movement.LEFTSTRAFE, 72);
                break;

            case 3:
                robot.encoderRun(Movement.DOWNLEFT, 72);
                break;

            case 4:
                robot.encoderRun(Movement.DOWNRIGHT, 72);
                break;

            case 5:
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                break;
        }
        telemetry.addData("Case:", robot.command);
        telemetry.addData("Runtime:", robot.runtime.milliseconds());
        telemetry.addData("Power:", robot.power);
        telemetry.update();
    }
}

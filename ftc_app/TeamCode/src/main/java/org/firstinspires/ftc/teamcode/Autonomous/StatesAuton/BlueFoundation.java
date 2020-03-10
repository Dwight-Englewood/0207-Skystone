package org.firstinspires.ftc.teamcode.Autonomous.StatesAuton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.Autonomous.Methods.NewAutonMethods;
import org.firstinspires.ftc.teamcode.Hardware.Movement;

@Autonomous(name = "Blue Foundation", group = "Autonomous")
public class BlueFoundation extends OpMode {
    NewAutonMethods robot = new NewAutonMethods();

    public void init() {
        robot.initNew(hardwareMap, telemetry); // init all ur motors and crap (NOTE: DO NOT INIT GYRO OR VISION IN THIS METHOD)

        new Thread()  {
            public void run() {
                robot.initGyro();
                telemetry.addData("Gyro Initialized", robot.isGyroInit());
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
                robot.setTarget(Movement.DOWNRIGHT, 200);
                break;

            case 1:
                robot.finishDrive();
                break;

            case 2:
                robot.newCloseServoAuton();
                if (robot.runtime.milliseconds() > 450) {
                    robot.command++;
                }
                break;

            case 3:
                robot.setTarget(Movement.FORWARD, 50);
                break;

            case 4:
                robot.finishDrive();
                break;

            case 5:
                robot.setTarget(Movement.UPLEFT, 90);
                break;

            case 6:
                robot.finishDrive();
                break;

            case 7:
                robot.gyroTurn(90);
                break;

            case 8:
                robot.setTarget(Movement.BACKWARD , 30);
                break;

            case 9:
                robot.newOpenServoAuton();
                robot.finishDrive();
                break;

            case 10:
                robot.setTarget(Movement.FORWARD,120);
                break;

            case 11:
                robot.finishDrive();
                break;
        }
        telemetry.addData("Case:", robot.command);
        telemetry.update();
    }
}


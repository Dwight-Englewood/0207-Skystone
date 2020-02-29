package org.firstinspires.ftc.teamcode.Autonomous.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.Autonomous.Methods.NewAutonMethods;
import org.firstinspires.ftc.teamcode.Hardware.Movement;

@Autonomous(name = "pidTest (Test)", group = "Autonomous")
public class pidTest extends OpMode {
    NewAutonMethods robot = new NewAutonMethods();

    public void init() {
  //      robot.initTest(hardwareMap, telemetry); // init all ur motors and crap (NOTE: DO NOT INIT GYRO OR VISION IN THIS METHOD)
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
         /*   case 0:
                robot.runMotor(Movement.FORWARD, 20);
                break;

            case 1:
                robot.finishMotor();
                robot.runtime.reset();
                break;

            case 2:
                if (robot.runtime.milliseconds() > 0) {
                    robot.runMotor(Movement.BACKWARD, 50);
                }
                break;

            case 3:
                robot.finishMotor();
                robot.runtime.reset();
                break;

            case 4:
                if (robot.runtime.milliseconds() > 0) {
                    robot.runMotor(Movement.FORWARD, 150);
                }
                break;

            case 5:
                robot.finishMotor();
                robot.runtime.reset();
                break;

            case 6:
                if (robot.runtime.milliseconds() > 0) {
                    robot.runMotor(Movement.BACKWARD, 300);
                }
                break;

            case 7:
                robot.finishMotor();
                robot.runtime.reset();
                break;

            case 8:
                telemetry.addLine("Code End");
                break;

          */
        }
        telemetry.addData("Case", robot.command);
        telemetry.update();
    }
}


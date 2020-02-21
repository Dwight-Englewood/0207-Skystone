package org.firstinspires.ftc.teamcode.Autonomous.NewActiveAuton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.Autonomous.Methods.NewAutonMethods;
import org.firstinspires.ftc.teamcode.Hardware.Movement;

@Autonomous(name = "TimerProgram", group = "Autonomous")
public class timerTest extends OpMode {
    NewAutonMethods robot = new NewAutonMethods();

    public void init() {
        robot.init(hardwareMap, telemetry); // init all ur motors and crap (NOTE: DO NOT INIT GYRO OR VISION IN THIS METHOD)
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
                robot.intakeL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.command++;
                break;

            case 1:
                robot.runtime.reset();
                robot.command++;
                break;

            case 2:
                if (robot.runtime.milliseconds() >= 3000){
                    robot.intakeL.setPower(0);
                    robot.command++;
                } else {
                    robot.intakeL.setPower(1);
                }
                break;

            case 3:
                telemetry.addLine("End of Program");
                break;
        }
        telemetry.addData("Case:", robot.command);
        telemetry.addData("Power:", robot.intakeL.getPower());
        telemetry.addData("Runtime:", robot.runtime.milliseconds());
        telemetry.update();
    }
}



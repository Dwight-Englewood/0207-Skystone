package org.firstinspires.ftc.teamcode.Autonomous.tests;

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

@Autonomous(name = "RetireProgram", group = "Autonomous")
public class ReitreProgram extends OpMode {
    NewAutonMethods robot = new NewAutonMethods();
    SkystoneDetect detector = new SkystoneDetect();

    public void init() {
        robot.initNew(hardwareMap, telemetry); // init all ur motors and crap (NOTE: DO NOT INIT GYRO OR VISION IN THIS METHOD)
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
        robot.liftLeftClaws();
        robot.liftRightClaws();
        robot.closeRightClaws();
        robot.closeLeftClaws();
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
                robot.lowerOpenRightClaws();
                robot.liftLeftClaws();
                robot.openLeftClaws();

                if(robot.runtime.milliseconds() > 3000){
                    robot.command++;
                }
                break;

            case 1:
                robot.lowerOpenLeftClaws();
                robot.liftRightClaws();
                robot.openRightClaws();

                if(robot.runtime.milliseconds() > 3000){
                    robot.command++;
                }
                break;

            case 2:
                robot.lowerOpenRightClaws();
                robot.liftLeftClaws();
                robot.openLeftClaws();

                if(robot.runtime.milliseconds() > 3000){
                    robot.command++;
                }
                break;

            case 3:
                robot.lowerOpenLeftClaws();
                robot.liftRightClaws();
                robot.openRightClaws();

                if(robot.runtime.milliseconds() > 3000){
                    robot.command++;
                }
                break;

            case 4:
                robot.lowerOpenRightClaws();
                robot.liftLeftClaws();
                robot.openLeftClaws();

                if(robot.runtime.milliseconds() > 3000){
                    robot.command++;
                }
                break;

            case 5:
                robot.lowerOpenLeftClaws();
                robot.liftRightClaws();
                robot.openRightClaws();

                if(robot.runtime.milliseconds() > 3000){
                    robot.command++;
                }
                break;
        }
        telemetry.addData("Case:", robot.command);
        telemetry.update();
    }
}

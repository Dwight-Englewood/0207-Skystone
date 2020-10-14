package org.firstinspires.ftc.teamcode.Autonomous.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Autonomous.Methods.NewAutonMethods;
import org.firstinspires.ftc.teamcode.Hardware.Movement;

@Autonomous(name = "PID Test", group = "Autonomous")
public class TestMet extends OpMode {
    NewAutonMethods robot = new NewAutonMethods();

    public void init() {
        robot.initNew(hardwareMap, telemetry); // init all ur motors and crap (NOTE: DO NOT INIT GYRO OR VISION IN THIS METHOD)

        new Thread() {
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
             //   robot.setHeadingTarget(Movement.FORWARD, 30, 90);
                break;

            case 1:
             //   robot.targetHeadingWrapper();
                break;

            case 2:
                robot.setHeadingTarget(Movement.BACKWARD, 30);
                break;

            case 3:
                robot.targetHeadingWrapper(0, 30);
                break;
        }
        telemetry.addData("Case:", robot.command);
        telemetry.addData("FL Power", robot.FL.getPower());
        telemetry.addData("FL TARGET", robot.FL.getTargetPosition());
        telemetry.addData("FL CURRENT", robot.FL.getCurrentPosition());

        telemetry.addData("CURRENT", robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);

        telemetry.addData("P", robot.error * robot.kpVal);
        telemetry.addData("I", robot.errorI * robot.kiVal);
        telemetry.addData("D", -robot.errorD * robot.kdVal);

        telemetry.addData("TP", robot.Terror * robot.TkpVal);
        telemetry.addData("TI", robot.TerrorI * robot.TkiVal);
        telemetry.addData("TD", -robot.TerrorD * robot.TkdVal);

        telemetry.update();
    }
}


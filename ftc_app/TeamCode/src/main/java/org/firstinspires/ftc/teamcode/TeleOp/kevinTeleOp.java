package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Hardware.DeuxBoot;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Kevin Tele-Op",group="TeleOp")
public class kevinTeleOp extends OpMode {
    DeuxBoot robot = new DeuxBoot();
    double speed;
    boolean buttonAheld = false;
    boolean grabberClosed = true;
    boolean buttonBheld = false;
    boolean hingedClosed = true;
    boolean buttonXheld = false;
    boolean spinnerClosed = false;
    boolean buttonYheld = false;
    boolean foundationClosed = false;

    @Override
    public void init() {
        robot.initNew(hardwareMap);

        new Thread()  {
            public void run() {
                robot.initGyro();
                robot.isGyroInit();// whatever ur init gyro method is on robot
            }
        }.start();

        robot.leftBlue.setPosition(0);
        robot.leftPurp.setPosition(1);
        robot.rightBlue.setPosition(1);
        robot.rightPurp.setPosition(0);
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        robot.runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the r hits PLAY but before they hit STOP
     */

    @Override
    public void loop() {
        robot.tankDrive(gamepad1.left_stick_x,
                gamepad1.left_stick_y,
                gamepad1.right_stick_x,
                gamepad1.left_trigger,
                gamepad1.right_trigger);

/*        telemetry.addData("FL Power", robot.FL.getPower());
        telemetry.addData("FR Power", robot.FR.getPower());
        telemetry.addData("BL Power", robot.BL.getPower());
        telemetry.addData("BR Power", robot.BR.getPower());

        telemetry.addData("Heading", robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
        telemetry.addData("+45",   Math.sin(robot.returnHeading() + 45));
        telemetry.addData("-45",   Math.sin(robot.returnHeading() - 45));
        telemetry.update();

 */
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */

    @Override
    public void stop () {
    }
}
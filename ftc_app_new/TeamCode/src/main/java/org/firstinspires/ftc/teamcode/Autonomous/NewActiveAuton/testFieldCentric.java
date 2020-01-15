package org.firstinspires.ftc.teamcode.Autonomous.NewActiveAuton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Autonomous.Methods.NewAutonMethods;
import org.firstinspires.ftc.teamcode.Hardware.Movement;

@Autonomous(name = "Field Centric", group = "Autonomous")
public class testFieldCentric extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DigitalChannel DigChannel;
    NewAutonMethods robot = new NewAutonMethods();

    public void init() {
        robot.init(hardwareMap, telemetry);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        robot.BR.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.BL.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.FL.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.FR.setDirection(DcMotorSimple.Direction.FORWARD);

        robot.FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        switch (robot.command) {
            case 0:
                robot.setTarget(25,25);
                break;

            case 1:
                robot.oldX = 25;
                robot.oldY = 25;
                robot.command++;
                break;

            case 2:
                robot.setTarget(25,-25);
                break;

            case 3:
                robot.oldX = 25;
                robot.oldY = -25;
                robot.command++;
                break;
        }
        telemetry.addData("Case:", robot.command);
        telemetry.update();
    }
}

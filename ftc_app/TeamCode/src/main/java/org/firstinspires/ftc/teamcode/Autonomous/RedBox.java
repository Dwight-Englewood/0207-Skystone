package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Boot;
import org.firstinspires.ftc.teamcode.Hardware.*;

//hello

@Autonomous(name = "redBox", group = "Autonomous")
public class RedBox extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DigitalChannel DigChannel;
    Boot robot = new Boot();

    int auto = 0;

    public void init() {
        robot.init(hardwareMap, telemetry, false);
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
        switch (auto) {
            case 0:
                try {
                    Thread.sleep(5000);
                } catch (Exception e) {
                    telemetry.addLine("Sleep Failed");
                    telemetry.update();
                }
                auto++;
                break;

            case 1:
                robot.autonDriveUltimate(Movement.LEFTSTRAFE, 2500, .5);
                auto++;
                break;

            case 2:

                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                auto++;
                break;

            case 3:
                robot.autonDriveUltimate(Movement.FORWARD, 1000, .5);
                auto++;
                break;

            case 4:

                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                //robot.leftServ.setPosition(0.5);
                //robot.rightServ.setPosition(0.5);
                auto++;
                break;

            case 5:
                robot.autonDriveUltimate(Movement.BACKWARD, 1000, .5);
                auto++;
                break;

            case 6:
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                //robot.rightServ.setPosition(0);
                //robot.leftServ.setPosition(0);
                auto++;
                break;

            case 7:
                robot.autonDriveUltimate(Movement.RIGHTSTRAFE, 1500, .5);
                auto++;
                break;

            case 8:
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                auto++;
                break;

        }
        telemetry.addData("Case Number:", auto);
        telemetry.update();
    }
}
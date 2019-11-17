package org.firstinspires.ftc.teamcode.Autonomous.Inactive;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.*;
import org.firstinspires.ftc.teamcode.Autonomous.*;

@Disabled
@Autonomous(name = "Single Sample Blue", group = "Autonomous")
public class BlueSingleSample extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DigitalChannel DigChannel;
    AutonMethods robot = new AutonMethods();

    int auto = 0;
    int stroll = 500;
    int block;

    public static Servo clamp;

    public void init() {
        robot.init(hardwareMap, telemetry, false);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        clamp = this.hardwareMap.get(Servo.class, "clamp");

        robot.BR.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.BL.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.FL.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.FR.setDirection(DcMotorSimple.Direction.FORWARD);

        robot.FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        clamp.setPosition(1);
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
                robot.autonDriveUltimate(Movement.FORWARD, 1200, 0.5);
                if (Math.abs(robot.FL.getCurrentPosition()) >= Math.abs(robot.FL.getTargetPosition())){
                    auto++;
                }
                break;
//Block 1
            case 1:
                try {
                    Thread.sleep(2000);
                } catch (InterruptedException E) {
                    telemetry.addLine("Sleep Failed");
                }
                robot.color_sensor.enableLed(true);
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                if(robot.color_sensor.red() <= 70 && robot.color_sensor.green() <= 70) {
                    auto = 98;
                    block = 1;
                } else {
                    auto++;
                }
                break;
//Block 2
            case 2:
                robot.autonDriveUltimate(Movement.LEFTSTRAFE, stroll, 0.5);
                if (Math.abs(robot.FL.getCurrentPosition()) >= Math.abs(robot.FL.getTargetPosition())){
                    if(robot.color_sensor.red() <= 70 && robot.color_sensor.green() <= 70) {
                        auto = 98;
                    } else {
                        auto++;
                    }
                }
                break;

            case 3:
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                auto++;
                break;
//Block 3
            case 4:
                robot.autonDriveUltimate(Movement.LEFTSTRAFE, stroll, 0.5);
                if (Math.abs(robot.FL.getCurrentPosition()) >= Math.abs(robot.FL.getTargetPosition())){
                    if(robot.color_sensor.red() <= 70 && robot.color_sensor.green() <= 70) {
                        auto = 98;
                    } else {
                        auto++;
                    }
                }
                break;

            case 5:
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                auto++;
                break;
//Block 4
            case 6:
                robot.autonDriveUltimate(Movement.LEFTSTRAFE, stroll, 0.5);
                if (Math.abs(robot.FL.getCurrentPosition()) >= Math.abs(robot.FL.getTargetPosition())){
                    if(robot.color_sensor.red() <= 70 && robot.color_sensor.green() <= 70) {
                        auto = 98;
                    } else {
                        auto++;
                    }
                }
                break;

            case 7:
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                auto++;
                break;

//Block 5
            case 8:
                robot.autonDriveUltimate(Movement.LEFTSTRAFE, stroll, 0.5);
                if (Math.abs(robot.FL.getCurrentPosition()) >= Math.abs(robot.FL.getTargetPosition())){
                    if(robot.color_sensor.red() <= 70 && robot.color_sensor.green() <= 70) {
                        auto = 98;
                    }
                }
                break;
//Block 6

            case 98:
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                auto++;
                break;

            case 99:
                robot.autonDriveUltimate(Movement.LEFTSTRAFE, 150, 0.5);
                if (Math.abs(robot.FL.getCurrentPosition()) >= Math.abs(robot.FL.getTargetPosition())){
                    auto++;
                }
                break;

            case 100:
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                auto++;
                break;

            case 101:
                this.clamp.setPosition(0);
                robot.autonDriveUltimate(Movement.BACKWARD, 150, 0.5);
                if (Math.abs(robot.FL.getCurrentPosition()) >= Math.abs(robot.FL.getTargetPosition())){
                    auto++;
                }
                break;

            case 102:
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                auto++;
                break;

            case 103:
                robot.autonDriveUltimate(Movement.LEFTSTRAFE, 3000, 0.5);
                if (Math.abs(robot.FL.getCurrentPosition()) >= Math.abs(robot.FL.getTargetPosition())){
                    auto++;
                }
                break;

            case 104:
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                auto++;

            case 105:
                robot.autonDriveUltimate(Movement.RIGHTSTRAFE, 3500, 0.5);
                if (Math.abs(robot.FL.getCurrentPosition()) >= Math.abs(robot.FL.getTargetPosition())){
                    auto++;
                }
                break;

            case 106:
                this.clamp.setPosition(1);
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                break;
        }
        telemetry.addData("Case:", auto);
        telemetry.addData("Red Val", robot.color_sensor.red());
        telemetry.addData("Green Val", robot.color_sensor.green());
        telemetry.update();
    }
}

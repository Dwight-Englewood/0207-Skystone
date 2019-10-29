package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.teamcode.Hardware.Boot;
import org.firstinspires.ftc.teamcode.Hardware.*;

//hello

@Autonomous(name = "Annoying Red (Test)", group = "Autonomous")
public class RedLeft extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DigitalChannel DigChannel;
    Boot robot = new Boot();

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
                robot.autonDriveUltimate(Movement.FORWARD, 500, 0.7);
                if (Math.abs(robot.FL.getCurrentPosition()) >= Math.abs(robot.FL.getTargetPosition())){
                    auto++;
                }
                break;

            case 1:
                robot.color_sensor.enableLed(true);
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                auto++;
                break;
//Block 1
            case 2:
                robot.autonDriveUltimate(Movement.LEFTSTRAFE, 1000, 0.7);
                if (Math.abs(robot.FL.getCurrentPosition()) >= Math.abs(robot.FL.getTargetPosition())){
                    if(robot.color_sensor.red() <= 70 && robot.color_sensor.green() <= 70) {
                        auto = 100;
                        block = 1;
                    } else {
                        auto++;
                    }
                }
                break;

            case 3:
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                auto++;
                break;
//Block 2
            case 4:
                robot.autonDriveUltimate(Movement.LEFTSTRAFE, stroll, 0.7);
                if(robot.color_sensor.red() <= 70 && robot.color_sensor.green() <= 70) {
                    auto = 100;
                    block = 2;
                } else {
                    auto++;
                }
                break;

            case 5:
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                auto++;
                break;
//Block 3
            case 6:
                robot.autonDriveUltimate(Movement.LEFTSTRAFE, stroll, 0.7);
                if(robot.color_sensor.red() <= 70 && robot.color_sensor.green() <= 70) {
                    auto = 100;
                    block = 3;
                } else {
                    auto++;
                }
                break;

            case 7:
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                auto++;
                break;

//Block 4
            case 8:
                robot.autonDriveUltimate(Movement.LEFTSTRAFE, stroll, 0.7);
                if(robot.color_sensor.red() <= 70 && robot.color_sensor.green() <= 70) {
                    auto = 100;
                    block = 4;
                } else {
                    auto++;
                }
                break;

            case 9:
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                auto++;
                break;
//Block 5
            case 10:
                robot.autonDriveUltimate(Movement.LEFTSTRAFE, stroll, 0.7);
                if(robot.color_sensor.red() <= 70 && robot.color_sensor.green() <= 70) {
                    auto = 100;
                    block = 5;
                } else {
                    auto++;
                }
                break;

            case 11:
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                auto++;
                break;
//Block 6
            case 12:
                robot.autonDriveUltimate(Movement.LEFTSTRAFE, stroll, 0.7);
                if(robot.color_sensor.red() <= 70 && robot.color_sensor.green() <= 70) {
                    auto = 100;
                    block = 6;
                } else {
                    auto = 100;
                    block = 6;
                }
                break;

/////////////////////////////////////////////////////////////////////////////////////////////////

            case 100:
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                auto++;
                break;

            case 101:
                robot.autonDriveUltimate(Movement.LEFTSTRAFE, 2000, 0.7);
                if (Math.abs(robot.FL.getCurrentPosition()) >= Math.abs(robot.FL.getTargetPosition())){
                    this.clamp.setPosition(0);
                    auto++;
                }
                break;

            case 102:
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                auto++;
                break;

            case 103:
                robot.autonDriveUltimate(Movement.BACKWARD, 500, 0.7);
                this.clamp.setPosition(0);
                if (Math.abs(robot.FL.getCurrentPosition()) >= Math.abs(robot.FL.getTargetPosition())){
                    auto++;
                }
                break;

            case 104:
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                auto = 700;
                break;

           /* case 105:
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                if (block == 1) {
                    auto = 200;
                } else if (block == 2) {
                    auto = 300;
                } else if (block == 3) {
                    auto = 400;
                } else if (block == 4) {
                    auto = 500;
                } else if (block == 5) {
                    auto = 600;
                } else if (block == 6) {
                    auto = 700;
                } else {
                    auto ++;
                }
                auto = 700;
                break;

            case 200:
                robot.autonDriveUltimate(Movement.RIGHTSTRAFE, 2500, 0);
                auto = 1000;
                break;

            case 300:
                robot.autonDriveUltimate(Movement.RIGHTSTRAFE, 2600, 0);
                auto = 1000;
                break;

            case 400:
                robot.autonDriveUltimate(Movement.RIGHTSTRAFE, 2700, 0);
                auto = 1000;
                break;

            case 500:
                robot.autonDriveUltimate(Movement.RIGHTSTRAFE, 2800, 0);
                auto = 1000;
                break;

            case 600:
                robot.autonDriveUltimate(Movement.RIGHTSTRAFE, 2900, 0);
                auto = 1000;
                break;

            */

            case 700:
                robot.autonDriveUltimate(Movement.RIGHTSTRAFE, 3000, 0);
                auto = 1000;
                break;


            case 1000:
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                auto++;
                break;


            case 1001:
                robot.autonDriveUltimate(Movement.FORWARD, 500, 0);
                auto++;
                break;

            case 1002:
                this.clamp.setPosition(0.6);
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                auto++;
                break;

            case 1003:
                robot.autonDriveUltimate(Movement.BACKWARD, 500, 0);
                auto++;
                break;

            case 1004:
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                auto++;
                break;

            case 1005:
                robot.autonDriveUltimate(Movement.LEFTSTRAFE, 1500, 0);
                auto++;
                break;

                /*
                if (Math.abs(45 - robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle) > 3) {
                    robot.adjustHeading(0);
                } else if (Math.abs(45 - robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle) < 3) {
                    robot.drive(Movement.STOP, 0);
                    auto++;
                }
                break;

                 */
        }
        telemetry.addData("Case Number:", auto);
        telemetry.addData("Gyro:", robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
        telemetry.update();
    }
}
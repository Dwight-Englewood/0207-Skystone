package org.firstinspires.ftc.teamcode.Autonomous.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Autonomous.Methods.WebVu;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.teamcode.Autonomous.Methods.AutonMethods;
@Disabled
@Autonomous(name = "Double Sample Red", group = "Autonomous")
public class RedDoubleSample extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DigitalChannel DigChannel;
    AutonMethods robot = new AutonMethods();
    WebVu tensorFlow = new WebVu();

    WebVu.TFState fakeState, actualState;

    int stroll = 500;
    int block;

    int count = 0;

    public static Servo clamp;

    public void init() {
        robot.init(hardwareMap, telemetry, false);
        tensorFlow.init(hardwareMap, telemetry);
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
        tensorFlow.start();
        fakeState = tensorFlow.getState();
    }
    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        switch (robot.command = 999) {
            case 999:
                fakeState = (fakeState != WebVu.TFState.NOTVISIBLE) ? fakeState : tensorFlow.getState();
                if(fakeState != WebVu.TFState.NOTVISIBLE){
                    telemetry.addLine("Visible");
                }
                else {
                    fakeState = WebVu.TFState.RIGHT;
                    telemetry.addLine("Failed, default to right");
                }
/*
            case 0:
                robot.runToTarget(Movement.FORWARD, 400, false);
                break;
//Block 1
            case 1:
                robot.color_sensor.enableLed(true);
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                if(robot.color_sensor.red() <= 70 && robot.color_sensor.green() <= 70) {
                    robot.command = 100;
                    block = 1;
                } else {
                    robot.command++;
                }
                break;
//Block 2
            case 2:
                robot.runToTarget(Movement.LEFTSTRAFE, 1000, true);
                if (Math.abs(robot.FL.getCurrentPosition()) >= Math.abs(robot.FL.getTargetPosition())){
                    if(robot.color_sensor.red() <= 70 && robot.color_sensor.green() <= 70) {
                        robot.command = 100;
                        block = 2;
                    } else {
                        robot.command++;
                    }
                }
                break;

            case 3:
                robot.encoderReset();
                break;
//Block 3
            case 4:
                robot.runToTarget(Movement.LEFTSTRAFE, stroll, true);
                if(robot.color_sensor.red() <= 70 && robot.color_sensor.green() <= 70) {
                    robot.command = 100;
                    block = 3;
                } else {
                    robot.command++;
                }
                break;

            case 5:
                robot.encoderReset();
                break;
//Block 4
            case 6:
                robot.runToTarget(Movement.LEFTSTRAFE, stroll, true);
                if(robot.color_sensor.red() <= 70 && robot.color_sensor.green() <= 70) {
                    robot.command = 100;
                    block = 4;
                } else {
                    robot.command++;
                }
                break;

            case 7:
                robot.encoderReset();
                break;

//Block 5
            case 8:
                robot.runToTarget(Movement.LEFTSTRAFE, stroll, true);
                if(robot.color_sensor.red() <= 70 && robot.color_sensor.green() <= 70) {
                    robot.command = 100;
                    block = 5;
                } else {
                    robot.command++;
                }
                break;

            case 9:
                robot.encoderReset();
                break;
//Block 6
            case 10:
                robot.runToTarget(Movement.LEFTSTRAFE, stroll, true);
                if(robot.color_sensor.red() <= 70 && robot.color_sensor.green() <= 70) {
                    robot.command = 100;
                    block = 6;
                } else {
                    robot.command = 100;
                }
                break;

            case 100:
                robot.runToTarget(Movement.RIGHTSTRAFE, 200, true);
                break;

            case 101:
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                this.clamp.setPosition(0);
                count++;
                break;

            case 102:
                robot.runToTarget(Movement.LEFTSTRAFE, 1500, true);
                break;

            case 103:
                this.clamp.setPosition(0);
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.command = 700;
                break;

            case 700:
                robot.runToTarget(Movement.RIGHTSTRAFE, 2500, true);
                break;


            case 701:
                this.clamp.setPosition(1);
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                if (count <= 1) {
                    robot.command++;
                } else {
                    robot.command = 2000;
                }
                break;


            case 702:
                robot.runToTarget(Movement.LEFTSTRAFE, 2500, true);
                break;

            case 703:
                robot.encoderReset();
                break;

            //Block 1
            case 704:
                robot.color_sensor.enableLed(true);
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                if(robot.color_sensor.red() <= 70 && robot.color_sensor.green() <= 70) {
                    robot.command = 100;
                    block = 1;
                } else {
                    robot.command++;
                }
                break;
//Block 2
            case 705:
                robot.runToTarget(Movement.RIGHTSTRAFE, 1000, true);
                if (Math.abs(robot.FL.getCurrentPosition()) >= Math.abs(robot.FL.getTargetPosition())){
                    if(robot.color_sensor.red() <= 70 && robot.color_sensor.green() <= 70) {
                        robot.command = 100;
                        block = 2;
                    } else {
                        robot.command++;
                    }
                }
                break;

            case 706:
                robot.encoderReset();
                break;
//Block 3
            case 707:
                robot.runToTarget(Movement.RIGHTSTRAFE, stroll, true);
                if(robot.color_sensor.red() <= 70 && robot.color_sensor.green() <= 70) {
                    robot.command = 100;
                    block = 3;
                } else {
                    robot.command++;
                }
                break;

            case 708:
                robot.encoderReset();
                break;
//Block 4
            case 709:
                robot.runToTarget(Movement.RIGHTSTRAFE, stroll, true);
                if(robot.color_sensor.red() <= 70 && robot.color_sensor.green() <= 70) {
                    robot.command = 100;
                    block = 4;
                } else {
                    robot.command++;
                }
                break;

            case 710:
                robot.encoderReset();
                break;

//Block 5
            case 711:
                robot.runToTarget(Movement.RIGHTSTRAFE, stroll, true);
                if(robot.color_sensor.red() <= 70 && robot.color_sensor.green() <= 70) {
                    robot.command = 100;
                    block = 5;
                } else {
                    robot.command++;
                }
                break;

            case 712:
                robot.encoderReset();
                break;
//Block 6
            case 713:
                robot.runToTarget(Movement.RIGHTSTRAFE, stroll, true);
                if(robot.color_sensor.red() <= 70 && robot.color_sensor.green() <= 70) {
                    robot.command = 100;
                    block = 6;
                } else {
                    robot.command = 100;
                }
                break;

            case 2000:
                robot.runToTarget(Movement.LEFTSTRAFE, 500, true);
                break;

            case 2001:
                robot.encoderReset();
                break;

 */
        }
        telemetry.addData("Case:", robot.command);
        telemetry.update();
    }
}

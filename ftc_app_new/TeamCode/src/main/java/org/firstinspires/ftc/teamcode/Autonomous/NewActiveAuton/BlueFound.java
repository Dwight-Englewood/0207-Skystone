package org.firstinspires.ftc.teamcode.Autonomous.NewActiveAuton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Autonomous.Methods.NewAutonMethods;
import org.firstinspires.ftc.teamcode.Hardware.*;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

@Autonomous(name = "BFound", group = "Autonomous")
public class BlueFound extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DigitalChannel DigChannel;
    NewAutonMethods robot = new NewAutonMethods();

    int current;

    public void init() {
        robot.init(hardwareMap, telemetry);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
/*
        robot.BR.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.BL.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.FL.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.FR.setDirection(DcMotorSimple.Direction.FORWARD);
 */

        robot.FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.openServoAuton();
        //      robot.tape.setDirection(DcMotorSimple.Direction.FORWARD);
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
        robot.blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BREATH_BLUE);
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        switch (robot.command) {
            case 0:
                robot.encoderReset();
                break;

            case 1:
                runtime.reset();
                robot.runToTarget(Movement.LEFTSTRAFE, 71*1.5);
                break;

            case 2:
                robot.closeServoAuton();
                if (runtime.milliseconds() > 2000) {
                    robot.command++;
                }
                break;

            case 3:
                robot.encoderReset();
                break;

            case 4:
                robot.runToTarget(Movement.RIGHTSTRAFE, 72*1.125);
                break;

            case 5:
                robot.encoderReset();
                break;

            case 6:
                robot.gyroTurn(88);
                break;

            case 7:
                runtime.reset();
                robot.encoderReset();
                break;

            case 8:
                robot.openServoAuton();
                if (runtime.milliseconds() > 2000) {
                    robot.command++;
                }
                break;

            case 9:
                robot.encoderReset();
                break;

            case 10:
                robot.runToTarget(Movement.LEFTSTRAFE , 25*2);
                break;

            case 11:
                //robot.tape.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.encoderReset();
                break;

            case 12:
                /*if (runtime.milliseconds() > 10000) {
         //           robot.tapeExtend(4500,0.5);
                }*/
                robot.runToTarget(Movement.BACKWARD,20*2);
                break;

            case 13:
                robot.encoderReset();
                break;

            case 14:
                robot.runToTarget(Movement.RIGHTSTRAFE,110*1.375);
                break;

            case 15:
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                break;
        }
        telemetry.addData("Case:", robot.command);
        telemetry.update();
    }
}

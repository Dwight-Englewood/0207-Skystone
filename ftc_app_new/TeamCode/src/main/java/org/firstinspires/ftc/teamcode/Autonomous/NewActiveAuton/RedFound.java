package org.firstinspires.ftc.teamcode.Autonomous.NewActiveAuton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Autonomous.Methods.NewAutonMethods;
import org.firstinspires.ftc.teamcode.Hardware.Movement;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

@Autonomous(name = "RFound", group = "Autonomous")
public class RedFound extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DigitalChannel DigChannel;
    NewAutonMethods robot = new NewAutonMethods();

    int current;

    public void init() {
        robot.init(hardwareMap, telemetry);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

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
        robot.blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BREATH_RED);
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
                robot.runToTarget(Movement.FORWARD, 31);
                break;

            case 2:
                robot.encoderReset();
                break;

            case 3:
                runtime.reset();
                robot.runToTarget(Movement.LEFTSTRAFE, 71*1.5);
                break;

            case 4:
                robot.closeServoAuton();
                if (runtime.milliseconds() > 2000) {
                    robot.command++;
                }
                break;

            case 5:
                robot.encoderReset();
                break;

            case 6:
                robot.runToTarget(Movement.RIGHTSTRAFE, 72*1.125);
                break;

            case 7:
                robot.encoderReset();
                break;

            case 8:
                robot.gyroTurn(-88);
                break;

            case 9:
                runtime.reset();
                robot.encoderReset();
                break;

            case 10:
                robot.openServoAuton();
                if (runtime.milliseconds() > 2000) {
                    robot.command++;
                }
                break;

            case 11:
                robot.encoderReset();
                break;

            case 12:
                robot.runToTarget(Movement.LEFTSTRAFE , 15*2);
                break;

            case 13:
                //robot.tape.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.encoderReset();
                break;

            case 14:
                /*if (runtime.milliseconds() > 10000) {
         //           robot.tapeExtend(4500,0.5);
                }*/
                robot.runToTarget(Movement.FORWARD,20*2);
                break;

            case 15:
                robot.encoderReset();
                break;

            case 16:
                robot.runToTarget(Movement.RIGHTSTRAFE,110*1.25);
                break;

            case 17:
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                break;
        }
        telemetry.addData("Case:", robot.command);
        telemetry.update();
    }
}

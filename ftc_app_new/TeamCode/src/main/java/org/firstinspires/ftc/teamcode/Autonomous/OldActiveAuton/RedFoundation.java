package org.firstinspires.ftc.teamcode.Autonomous.OldActiveAuton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Autonomous.Methods.AutonMethods;
import org.firstinspires.ftc.teamcode.Hardware.*;

@Disabled
@Autonomous(name = "Red Foundation", group = "Autonomous")
public class RedFoundation extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DigitalChannel DigChannel;
    AutonMethods robot = new AutonMethods();

    public int auto = 0;

    int center = 150;
    int left = 600;
    int right = 350;

    int centerBack = 1100;
    int leftBack = 800;
    int rightBack = 1750;

    int curVal = 0;

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
        robot.LSERV.setDirection(Servo.Direction.REVERSE);
        robot.tape.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.openServo();
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
                this.clamp.setPosition(1);
                robot.runToTarget(Movement.BACKWARD, 30, false);
                break;

            case 1:
                robot.encoderReset();
                break;

            case 2:
                runtime.reset();
                robot.runToTarget(Movement.RIGHTSTRAFE, 73,  true);
                break;

            case 3:
                robot.closeServoAuton();
                if (runtime.milliseconds() > 2000) {
                    robot.command++;
                }
                break;

            case 4:
                robot.runToTarget(Movement.LEFTSTRAFE, 77,  true);
                break;

            case 5:
                runtime.reset();
                robot.gyroTurn(-90);
                break;

            case 6:
                robot.openServoAuton();
                if (runtime.milliseconds() > 2000) {
                    robot.command++;
                }
                break;

            case 7:
                robot.runToTarget(Movement.RIGHTSTRAFE , 40,  true);
                break;

            case 8:
                robot.tape.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.encoderReset();
                break;

            case 9:
                if (runtime.milliseconds() > 10000) {
                    robot.tapeExtend(4500,0.5);
                }
                break;

            case 10:
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                break;
        }
        telemetry.addData("Case:", robot.command);
        telemetry.update();
    }
}

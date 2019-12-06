package org.firstinspires.ftc.teamcode.Autonomous.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Autonomous.Methods.AutonMethods;
import org.firstinspires.ftc.teamcode.Hardware.Movement;
@Disabled
@Autonomous(name = "RedSuperAuton", group = "Autonomous")
public class RedSuperAuton extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DigitalChannel DigChannel;
    AutonMethods robot = new AutonMethods();

    public static Servo clamp;
    public static DcMotor lift;

    public void init() {
        robot.init(hardwareMap, telemetry, false);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        clamp = this.hardwareMap.get(Servo.class, "clamp");

        robot.BR.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.BL.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.FL.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.FR.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.lift.setDirection(DcMotorSimple.Direction.FORWARD);

        robot.FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
                robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                this.clamp.setPosition(1);
                robot.runToTarget(Movement.FORWARD, 71, false);
                break;

            case 1:
                robot.closeClampAuton();
                break;

            case 2:
                robot.runToTarget(Movement.BACKWARD, 20, false);
                break;

            case 3:
                robot.encoderReset();
                break;

            case 4:
                robot.runToTarget(Movement.RIGHTSTRAFE, 240,  true);
                break;

            case 5:
                robot.raiseLift(120);
                break;

            case 6:
                robot.liftReset();
                robot.runToTarget(Movement.FORWARD, 7,  false);
                break;

            case 7:
                robot.lowerLift(100);
                break;

            case 8:
                robot.liftReset();
                robot.openClampAuton();
                break;

            case 9:
                robot.runToTarget(Movement.BACKWARD, 7, false);
                break;

            case 10:
                robot.lowerLift(20);
                break;

            case 11:
                robot.liftReset();
                robot.runToTarget(Movement.LEFTSTRAFE, 300, true);
                break;

            case 12:
                robot.encoderReset();
                break;

            case 13:
                robot.runToTarget(Movement.BACKWARD, 50, false);
                break;

            case 14:
                robot.encoderReset();
                break;

            case 15:
                robot.runToTarget(Movement.FORWARD, 20, false);
                break;

            case 16:
                robot.closeClampAuton();
                break;

            case 17:
                robot.runToTarget(Movement.BACKWARD, 20, false);
                break;

            case 18:
                robot.encoderReset();
                break;

            case 19:
                robot.liftReset();
                robot.runToTarget(Movement.RIGHTSTRAFE, 240,  true);
                break;

            case 20:
                robot.raiseLiftDeux(140);
                break;

            case 21:
                robot.liftReset();
                robot.runToTarget(Movement.FORWARD, 7,  false);
                break;

            case 22:
                robot.lowerLiftDeux(120);
                break;

            case 23:
                robot.openClampAuton();
                break;

            case 24:
                robot.encoderReset();
                break;

            case 25:
                robot.runToTarget(Movement.BACKWARD, 7,  false);
                break;

            case 26:
                robot.turn(Movement.RIGHTTURN , 100,  0.25);
                break;

            case 27:
                robot.openServoAuton();
                break;

            case 28:
                robot.runToTarget(Movement.RIGHTSTRAFE , 10,  true);
                break;

            case 29:
                robot.closeServoAuton();
                break;

            case 30:
                robot.runToTarget(Movement.LEFTSTRAFE , 40,  true);
                break;

            case 31:
                robot.encoderReset();
                break;

            case 32:
                robot.runToTarget(Movement.LEFTTURN , 100,  false);
                break;

            case 33:
                robot.encoderReset();
                break;

            case 34:
                robot.runToTarget(Movement.RIGHTSTRAFE , 40,  true);
                break;

            case 35:
                robot.encoderReset();
                break;

            case 36:
                robot.runToTarget(Movement.FORWARD , 10,  false);
                break;

            case 37:
                robot.encoderReset();
                break;

            case 38:
                robot.runToTarget(Movement.LEFTSTRAFE , 100,  true);
                break;

            case 39:
                robot.liftReset();
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                break;
        }
        telemetry.addData("Case:", robot.command);
        telemetry.addData("Lift target", robot.lift.getTargetPosition());
        telemetry.addData("Lift current", robot.lift.getCurrentPosition());
        telemetry.update();
    }
}

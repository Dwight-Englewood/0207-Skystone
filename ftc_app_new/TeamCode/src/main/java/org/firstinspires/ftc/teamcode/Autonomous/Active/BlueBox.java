package org.firstinspires.ftc.teamcode.Autonomous.Active;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Autonomous.Methods.AutonMethods;
import org.firstinspires.ftc.teamcode.Hardware.Movement;
@Autonomous(name = "BlueBox", group = "Autonomous")
public class BlueBox extends OpMode {
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

        robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.LSERV.setDirection(Servo.Direction.REVERSE);
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
                robot.runToTarget(Movement.FORWARD, 71, false);
                break;

            case 1:
                robot.closeClampAuton();
                if (runtime.milliseconds() > 2000) {
                    robot.command++;
                }
                break;

            case 2:
                robot.runToTarget(Movement.BACKWARD, 65, false);
                break;

            case 3:
                robot.encoderReset();
                break;

            case 4:
                runtime.reset();

                robot.runToTarget(Movement.RIGHTSTRAFE, 152, true);
                break;

            case 5:
                robot.openClampAuton();
                if (runtime.milliseconds() > 1000) {
                    robot.command++;
                }
                break;

            case 6:
                robot.runToTarget(Movement.BACKWARD, 5, false);
                break;

            case 7:
                robot.encoderReset();
                break;

            case 8:
                robot.runToTarget(Movement.LEFTSTRAFE, 250, true);
                break;

            case 9:
                robot.encoderReset();
                break;

            case 10:
                robot.runToTarget(Movement.RIGHTSTRAFE, 20, true);
                break;

            case 11:
                robot.encoderReset();
                break;

            case 12:
                robot.runToTarget(Movement.BACKWARD, 10, false);
                break;

            case 13:
                robot.encoderReset();
                break;

            case 14:
                robot.runToTarget(Movement.FORWARD, 73, false);
                break;

            case 15:
                robot.encoderReset();
                break;

            case 16:
                runtime.reset();

                robot.runToTarget(Movement.RIGHTSTRAFE, 22, true);
                break;

            case 17:
                robot.closeClampAuton();
                if (runtime.milliseconds() > 2000) {
                    robot.command++;
                }
                break;

            case 18:
                robot.runToTarget(Movement.BACKWARD, 25, false);
                break;

            case 19:
                robot.encoderReset();
                break;

            case 20:
                robot.runToTarget(Movement.RIGHTSTRAFE, 150, true);
                break;

            case 21:
                robot.encoderReset();
                break;

            case 22:
                runtime.reset();

                robot.runToTarget(Movement.FORWARD, 20, false);
                break;

            case 23:
                robot.openClampAuton();
                if (runtime.milliseconds() > 1000) {
                    robot.command++;
                }
                break;

            case 24:
                robot.runToTarget(Movement.BACKWARD, 20, false);
                break;

            case 25:
                robot.encoderReset();
                break;

            case 26:
                robot.runToTarget(Movement.LEFTSTRAFE, 50, true);
                break;

            case 27:
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                break;
        }
        telemetry.addData("Case:", robot.command);
        telemetry.update();
    }
}

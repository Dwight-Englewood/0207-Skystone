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

@Autonomous(name = "RFound", group = "Autonomous")
public class RedFound extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DigitalChannel DigChannel;
    NewAutonMethods robot = new NewAutonMethods();

    public void init() {
        robot.init(hardwareMap, telemetry, false);
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
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        switch (robot.command) {
            case 0:
                robot.runToTarget(Movement.FORWARD, 31, false);
                break;

            case 1:
                robot.encoderReset();
                break;

            case 2:
                runtime.reset();
                robot.runToTarget(Movement.LEFTSTRAFE, 71,  true);
                break;

            case 3:
                robot.closeServoAuton();
                if (runtime.milliseconds() > 2000) {
                    robot.command++;
                }
                break;

            case 4:
                robot.runToTarget(Movement.RIGHTSTRAFE, 72,  true);
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
                robot.encoderReset();
                break;

            case 8:
                robot.runToTarget(Movement.LEFTSTRAFE , 15,  true);
                break;

            case 9:
         //       robot.tape.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.encoderReset();
                break;

            case 10:
                /*if (runtime.milliseconds() > 10000) {
         //           robot.tapeExtend(4500,0.5);
                }*/
                robot.runToTarget(Movement.FORWARD,20, true);
                break;

            case 11:
                robot.encoderReset();
                break;

            case 12:
                robot.runToTarget(Movement.RIGHTSTRAFE,110, true);
                break;

            case 13:
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                break;
        }
        telemetry.addData("Case:", robot.command);
        telemetry.update();
    }
}

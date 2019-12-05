package org.firstinspires.ftc.teamcode.Autonomous.Inactive;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Autonomous.Methods.AutonMethods;

@Disabled
@Autonomous(name = "BlueFoundationWall", group = "Autonomous")
public class BlueFoundationWall extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DigitalChannel DigChannel;
    AutonMethods robot = new AutonMethods();

    int auto = 0;

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
            /*
            case 0:
                this.clamp.setPosition(1);
                robot.openServo();
                robot.runToTarget(Movement.FORWARD, 500, 0.5);
                break;

            case 1:
                robot.encoderReset();
                break;

            case 2:
                robot.runToTarget(Movement.RIGHTSTRAFE, 1500, 0.35);
                break;

            case 3:
                robot.closeServ();
                break;

            case 4:
                robot.runToTarget(Movement.LEFTSTRAFE, 1600, 0.5);
                break;

            case 5:
                robot.encoderReset();
                break;

            case 6:
                robot.runToTarget(Movement.RIGHTTURN , 2000, 0.3);
                break;

            case 7:
                robot.openServ();
                break;

            case 8:
                robot.runToTarget(Movement.RIGHTSTRAFE , 500, 0.5);
                break;

            case 9:
                robot.encoderReset();
                break;

            case 10:
                robot.runToTarget(Movement.FORWARD , 450, 0.5);
                break;

            case 11:
                robot.encoderReset();
                break;

            case 12:
                robot.runToTarget(Movement.LEFTSTRAFE , 2000, 0.5);
                break;

            case 13:
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                break;

             */
        }
        telemetry.addData("Case:", auto);
        telemetry.update();
    }
}

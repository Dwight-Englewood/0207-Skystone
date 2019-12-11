package org.firstinspires.ftc.teamcode.TeleOp.Old;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Boot;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="MagTest [Delete]",group="TeleOp")
//Disabled
public class MagTest extends OpMode {
    // Declare OpMode members.
    private ElapsedTime timer = new ElapsedTime();
    Boot robot = new Boot();
    boolean wabbo = false;

    public static DcMotor lift;
    public static Servo clamp;
    double speed = 1;

    @Override
    public void init() {
        robot.init(hardwareMap, telemetry, false);
        telemetry.addData("Status", "Initialized");

        lift = this.hardwareMap.get(DcMotor.class, "Lift");
        clamp = this.hardwareMap.get(Servo.class, "clamp");

        lift.setDirection(DcMotorSimple.Direction.FORWARD);

        //   robot.color_sensor.enableLed(false);

        robot.BR.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.BL.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.FL.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.FR.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        timer.reset();
    }

    /*
     * Code to run REPEATEDLY after the r hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        robot.notKevinDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.left_trigger,gamepad1.right_trigger);
        if(robot.magSwitch.getState()){
            this.lift.setPower(gamepad2.right_stick_y*0.5);
        }
        else if(!robot.magSwitch.getState() && gamepad2.right_stick_y > 0){
            this.lift.setPower(gamepad2.right_stick_y);
        }
        else if(!robot.magSwitch.getState() && gamepad2.right_stick_y < 0){
            this.lift.setPower(0);
        } else {
            this.lift.setPower(0);
        }

        this.lift.setPower(gamepad2.right_stick_y);

        telemetry.addData("MagSwitch State", robot.magSwitch.getState());
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */


    @Override
    public void stop() {
    }

}

package org.firstinspires.ftc.teamcode.TeleOp.Old;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Boot;
@Disabled
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Tele-op (Motorcycle)",group="TeleOp")
public class MotorBot extends OpMode {
    private ElapsedTime timer = new ElapsedTime();
    Boot robot = new Boot();
    public static DcMotor front, back;

    @Override
    public void init() {
        robot.init(hardwareMap, telemetry, false);
        telemetry.addData("Status", "Initialized");

        front = this.hardwareMap.get(DcMotor.class, "Front");
        back = this.hardwareMap.get(DcMotor.class, "Back");

        this.front.setDirection(DcMotorSimple.Direction.FORWARD);
        this.back.setDirection(DcMotorSimple.Direction.FORWARD);
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
            front.setPower(gamepad1.left_stick_y);
            back.setPower(gamepad1.right_stick_y);
        }

    /*
     * Code to run ONCE after the driver hits STOP
     */


    @Override
    public void stop () {
    }
}

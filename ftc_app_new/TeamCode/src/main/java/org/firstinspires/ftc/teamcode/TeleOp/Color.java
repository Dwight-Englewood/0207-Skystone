package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Boot;
@Disabled
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Tele-op",group="TeleOp")
//Disabled
public class Color extends OpMode {
    // Declare OpMode members.
    private ElapsedTime timer = new ElapsedTime();
    Boot robot = new Boot();
    boolean wabbo = false;

    public static DcMotor lift;
    public static Servo clamp;
    double speed = 1;
    public ColorSensor color_sensor;


    @Override
    public void init() {
        robot.init(hardwareMap, telemetry, false);
        telemetry.addData("Status", "Initialized");

        lift = this.hardwareMap.get(DcMotor.class, "Lift");
        clamp = this.hardwareMap.get(Servo.class, "clamp");
        color_sensor = this.hardwareMap.get(ColorSensor.class, "Col");


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
        robot.color_sensor.enableLed(true);
        telemetry.addData("robot.color_sensor.red()", robot.color_sensor.alpha());
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */


    @Override
    public void stop () {
    }
}

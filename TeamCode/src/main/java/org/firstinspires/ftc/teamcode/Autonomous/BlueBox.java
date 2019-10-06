package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DigitalChannel;
//hello
import org.firstinspires.ftc.teamcode.Hardware.Movement;
import org.firstinspires.ftc.teamcode.Hardware.*;

import com.vuforia.CameraDevice;

@Autonomous(name="BB", group="Autonomous")
public class BlueBox extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DigitalChannel DigChannel;
    Boot robot = new Boot();

    int auto = 0;

    public void init() {
        robot.init(hardwareMap, telemetry, false);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        robot.BR.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.BL.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.FL.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.FR.setDirection(DcMotorSimple.Direction.FORWARD);

        robot.FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
       /* switch (auto) {
            case 0:
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                auto++;
                break;

            case 1:
                robot.autonDriveUltimate(Movement.LEFTSTRAFE, 1000, .5);
                auto++;
                break;

            case 2:

                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                auto++;
                break;

            case 3:
                robot.autonDriveUltimate(Movement.FORWARD, 560, .5);
                auto++;
                break;

            case 4:

                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                auto++;
                break;

            case 5:
                robot.autonDriveUltimate(Movement.FORWARD, 560, .5);
                auto++;
                break;

            case 6:

                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                auto++;
                break;


            case 7:
                robot.autonDriveUltimate(Movement.FORWARD, 560, .5);
                robot.intake.setTargetPosition(2000);
                robot.intake.setPower(1);
                robot.intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //      BigThonk = (tensorFlow.getState() == TensorFlow.TFState.NOTVISIBLE) ? BigThonk : tensorFlow.getState();

                if ( Math.abs(robot.intake.getCurrentPosition()) >= 2000) {
                    //  BigThonk = (BigThonk != TensorFlow.TFState.NOTVISIBLE) ? BigThonk : tensorFlow.getState();
                    robot.intake.setPower(0);
                    telemetry.update();
                    auto++;
                }
                break;

            case 8:
                robot.autonDriveUltimate(Movement.BACKWARD, 140, 0.5);
                if (Math.abs(robot.FL.getCurrentPosition()) >= Math.abs(robot.FL.getTargetPosition())){
                    auto++;
                }
                break;

            case 9:
                auto++;
                break;

            case 10:
                CameraDevice.getInstance().setFlashTorchMode(false);
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                auto++;
                break;

            case 11:
                robot.autonDriveUltimate(Movement.RIGHTSTRAFE, 280, 0.2);
                if (Math.abs(robot.FL.getCurrentPosition()) >= Math.abs(robot.FL.getTargetPosition())){
                    auto++;
                }
                break;

            case 12:
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                auto++;
                break;

            case 13:
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                auto = 14;
                break;

            case 14:
                robot.autonDriveUltimate(Movement.FORWARD, 500, 0.4);
                if (Math.abs(robot.FL.getCurrentPosition()) >= Math.abs(robot.FL.getTargetPosition())){
                    auto++;
                }
                break;

            case 15:
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                auto++;
                break;
        }
        telemetry.addData("Case Number:", auto);
        telemetry.update();*/
    }
}
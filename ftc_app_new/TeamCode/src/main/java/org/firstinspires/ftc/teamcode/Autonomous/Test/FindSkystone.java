package org.firstinspires.ftc.teamcode.Autonomous.Test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Autonomous.Methods.SkystoneDetect;

@TeleOp(name="find skystone TF", group="Iterative Opmode")
public class FindSkystone extends OpMode
{
    SkystoneDetect detector = new SkystoneDetect();
    public int left = 0;
    public int right = 0;
    public int middle = 0;
    public int notvis = 0;

    @Override
    public void init() {
        detector.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        detector.start();
    }


    @Override
    public void loop() {
        SkystoneDetect.Spot returnedloc = detector.getSkystonePosRed(telemetry);
        //SkystoneDetector.Spot returnedloc = detector.getSkystonePosRed(telemetry);

        switch (returnedloc) {
            case LEFT:
                left++;
                break;

            case RIGHT:
                right++;
                break;

            case MIDDLE:
                middle++;
                break;

            case NOTVISIBLE:
                notvis++;
                break;
        }
        telemetry.addData("left: ", left);
        telemetry.addData("middle: ", middle);
        telemetry.addData("right: ", right);
        telemetry.addData("notvis: ", notvis);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        detector.stop();
    }


}
package org.firstinspires.ftc.teamcode.Autonomous.Methods;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Autonomous.*;
import org.firstinspires.ftc.teamcode.Autonomous.NewActiveAuton.RBox;

import java.util.List;

public class SkystoneDetect {
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";
    private static final String VUFORIA_KEY = "AalCmlL/////AAABmX1QrqB1nUg9ocZ6eiXDvG0PvDRt0IXeA3yX89HHD+kI67mRqnF1LxjWbCI5xJwYIOLc5WxjOQ0mbCPV/wmDi2Cx6kjFBlcXtkrhAA6v8Ag5yslgW+7bB0JZv7+LnQcMG1u5rH+qBG3i4C0EeJHP73k+KzJ3RTS+j9c1uK0vlf+1NCuN6xPwnWRIRgM+SEYPxh2N96f77gWGCWlGn8cUN/sr28d4KIvdYU7yQJF1QzlWDf+53OcgpdzmV04mPmWb4hQS/a2LUCG8rTVMtjmqn5rfksJnQS7xWzxFeyymCmaTShJDescyOaRyokH7GF7vckPATFQiZGvMuVT4LheJoVMocEpdiTs86M7tvk06LogG";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    public enum Spot {
        LEFT, MIDDLE, RIGHT, NOTVISIBLE;
    }

    public boolean blue, red;

    public SkystoneDetect() {}

    public void init(HardwareMap hwMap) {
        this.initVuforia(hwMap);

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            this.initTfod(hwMap);
        }
    }

    public void start() {
        if (tfod != null) {
            tfod.activate();
        }
    }

    public void stop() {
        if (tfod != null) {
            tfod.shutdown();
        }
    }

    /*public int getNumVis() {
        if (tfod != null) {
            // Make this a var since this is a constantly changing thing and we want to check 1 instance
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                return updatedRecognitions.size();
            }
        }
        return 0;
    }*/

    // Created for looking at RIGHTMOST 2 BLOCKS (next to wall)
    public Spot getSkystonePosRed(Telemetry telemetry) {
        if (tfod != null) {
            // Make this a var since this is a constantly changing thing and we want to check 1 instance
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

            if (updatedRecognitions != null) {
                //telemetry.addData("# Object Detected", updatedRecognitions.size());
                for (Recognition recognition : updatedRecognitions) {
                    // is skystone visible?
                    if (recognition.getLabel().equals(LABEL_SECOND_ELEMENT)) {
                        // is skystone on left side of screen?
                        //TODO: Might need to change this to something else
                        if ((recognition.getRight() - recognition.getLeft())/2 + recognition.getLeft() < recognition.getImageWidth() / 2) {
                            return Spot.MIDDLE;
                        } else {
                            return Spot.RIGHT;
                        }
                    }
                }
                // Things visible, but no skystone, then must be right.
                return Spot.LEFT;
            }
        }
        // Nothing visible:
        return Spot.NOTVISIBLE;
    }

    public Spot getSkystonePos(Telemetry telemetry, boolean position) {
        if (position = blue) {
            if (tfod != null) {
                // Make this a var since this is a constantly changing thing and we want to check 1 instance
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

                if (updatedRecognitions != null) {
                    //telemetry.addData("# Object Detected", updatedRecognitions.size());
                    for (Recognition recognition : updatedRecognitions) {
                        // is skystone visible?
                        if (recognition.getLabel().equals(LABEL_SECOND_ELEMENT)) {
                            // is skystone on left side of screen?
                            //TODO: Might need to change this to something else
                            if ((recognition.getRight() - recognition.getLeft()) / 2 + recognition.getLeft() < recognition.getImageWidth() / 2) {
                                return Spot.LEFT;
                            } else {
                                return Spot.MIDDLE;
                            }
                        }
                    }
                    // Things visible, but no skystone, then must be right.
                    return Spot.RIGHT;
                }
            }
            // Nothing visible:
            return Spot.NOTVISIBLE;

        } else if (position = red) {
            if (tfod != null) {
                // Make this a var since this is a constantly changing thing and we want to check 1 instance
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

                if (updatedRecognitions != null) {
                    //telemetry.addData("# Object Detected", updatedRecognitions.size());
                    for (Recognition recognition : updatedRecognitions) {
                        // is skystone visible?
                        if (recognition.getLabel().equals(LABEL_SECOND_ELEMENT)) {
                            // is skystone on left side of screen?
                            //TODO: Might need to change this to something else
                            if ((recognition.getRight() - recognition.getLeft()) / 2 + recognition.getLeft() < recognition.getImageWidth() / 2) {
                                return Spot.MIDDLE;
                            } else {
                                return Spot.RIGHT;
                            }
                        }
                    }
                    // Things visible, but no skystone, then must be right.
                    return Spot.LEFT;
                }
            }
            // Nothing visible:
            return Spot.NOTVISIBLE;
        }
        return Spot.NOTVISIBLE;
    }

        // Created for looking at RIGHTMOST 2 BLOCKS (next to wall)
    public Spot getSkystonePosBlue(Telemetry telemetry) {
        if (tfod != null) {
            // Make this a var since this is a constantly changing thing and we want to check 1 instance
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

            if (updatedRecognitions != null) {
                //telemetry.addData("# Object Detected", updatedRecognitions.size());
                for (Recognition recognition : updatedRecognitions) {
                    // is skystone visible?
                    if (recognition.getLabel().equals(LABEL_SECOND_ELEMENT)) {
                        // is skystone on left side of screen?
                        //TODO: Might need to change this to something else
                        if ((recognition.getRight() - recognition.getLeft())/2 + recognition.getLeft() < recognition.getImageWidth() / 2) {
                            return Spot.LEFT;
                        } else {
                            return Spot.MIDDLE;
                        }
                    }
                }
                // Things visible, but no skystone, then must be right.
                return Spot.RIGHT;
            }
        }
        // Nothing visible:
        return Spot.NOTVISIBLE;
    }


    // COPIED FROM EXAMPLE:
    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia(HardwareMap hardwareMap) {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "cam");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod(HardwareMap hardwareMap) {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.8;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}

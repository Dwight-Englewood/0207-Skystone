package org.firstinspires.ftc.teamcode.Autonomous.Methods;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.WebcamConfiguration;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Hardware.State;
import org.firstinspires.ftc.teamcode.Hardware.Subsystem;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

public class WebVu implements Subsystem {

    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String BLACK_BLOCK = "Black Block";
    private static final String GOLD_BLOCK = "Gold Block";
    private static final String VUFORIA_KEY = "AalCmlL/////AAABmX1QrqB1nUg9ocZ6eiXDvG0PvDRt0IXeA3yX89HHD+kI67mRqnF1LxjWbCI5xJwYIOLc5WxjOQ0mbCPV/wmDi2Cx6kjFBlcXtkrhAA6v8Ag5yslgW+7bB0JZv7+LnQcMG1u5rH+qBG3i4C0EeJHP73k+KzJ3RTS+j9c1uK0vlf+1NCuN6xPwnWRIRgM+SEYPxh2N96f77gWGCWlGn8cUN/sr28d4KIvdYU7yQJF1QzlWDf+53OcgpdzmV04mPmWb4hQS/a2LUCG8rTVMtjmqn5rfksJnQS7xWzxFeyymCmaTShJDescyOaRyokH7GF7vckPATFQiZGvMuVT4LheJoVMocEpdiTs86M7tvk06LogG";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    Telemetry tele;
    HardwareMap hwMap;

    public WebVu() {}

    public enum TFState implements State {
        // crater angle, depot angle, crater distance, depot distance; all in degrees & cm
        LEFT("Left", 0, 0, 0, 0), //temp vals
        CENTER("Center", 0, 0, 0, 0), //temp vals
        RIGHT("Right", 0, 0, 0, 0), //temp vals
        NOTVISIBLE("None", 0, 0, 0, 0);

        private String str;
        private int craterAng;
        private int depotAng;
        private int craterDist;
        private int depotDist;

        TFState(String str, int craterAng, int depotAng, int craterDist, int depotDist) {
            this.str = str;
            this.craterAng = craterAng;
            this.depotAng = depotAng;
            this.craterDist = craterDist;
            this.depotDist = depotDist;
        }

        @Override
        public String toString() {
            return super.toString();
        }
        public int getCraterAng(){
            return craterAng;
        }
        public int getDepotAng(){
            return depotAng;
        }
        public int getCraterDist(){
            return craterDist;
        }
        public int getDepotDist(){
            return depotDist;
        }
    }

    private TFState state;

    @Override
    public void init(HardwareMap hwMap, Telemetry tele) {
        this.tele = tele;
        this.hwMap = hwMap;
        this.initVuforia();


        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            this.initTfod(hwMap);

            tele.addLine("TensorFlow Initiated");
        }
    }
    public List<Recognition> getD(){
        if(tfod != null){
            return tfod.getRecognitions();}
        else{return null;}

    }
    @Override
    public void start() {
        if (tfod != null) {
            tfod.activate();
            tele.addLine("TensorFlow Activated");
        }
    }

    @Override
    public void reset() {

    }

    @Override
    public void stop() {
        if (tfod != null) {
            tfod.shutdown();
        }
    }

    @Override
    public TFState getState() {
        this.updateState();
        return state;
    }

    private void updateState() {
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

            if (updatedRecognitions != null) {
                if (updatedRecognitions.size() == 2){
                    int black = -1;
                    int gold = -1;
                    // int silver2 = -1;
                    for(Recognition recognition: updatedRecognitions){
                        if(recognition.getLabel().equals(BLACK_BLOCK)){
                            black = (int) recognition.getBottom();
                        } else if(gold == -1 && recognition.getConfidence() > .6){
                            gold = (int) recognition.getBottom();
                        }
                    }
                    if (black != -1 || gold != -1) {
                        if (black < gold && black != -1){
                            this.state = TFState.LEFT;
                        }else if (gold < black) {
                            this.state = TFState.CENTER;
                        }else if (black == -1){
                            this.state = TFState.RIGHT;
                        }
                    }

                } else {
                    this.state = TFState.NOTVISIBLE;
                }
            }
        }
    }


    public void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hwMap.get(CameraName.class, "Webcam");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    public void initTfod(HardwareMap hardwareMap) {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, GOLD_BLOCK, BLACK_BLOCK);
    }
}

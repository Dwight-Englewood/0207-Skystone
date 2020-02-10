package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class GyroCalibration implements Subsystem {
    private BNO055IMU gyro;

    public GyroCalibration(){}

    @Override
    public void init(HardwareMap hwMap, Telemetry telemetry) {
  //      gyro = hwMap.get(BNO055IMU.class, "imu");
    }

    @Override
    public void start() {

    }

    @Override
    public void reset() {

    }

    @Override
    public void stop() {

    }

    @Override
    public State getState() {
        return null;
    }
}
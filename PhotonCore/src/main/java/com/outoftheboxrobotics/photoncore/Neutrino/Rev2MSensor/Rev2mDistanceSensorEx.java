package com.outoftheboxrobotics.photoncore.Neutrino.Rev2MSensor;

import com.qualcomm.hardware.lynx.LynxI2cDeviceSynch;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.TypeConversion;

import static com.qualcomm.hardware.stmicroelectronics.VL53L0X.Register.RESULT_RANGE_STATUS;

public class Rev2mDistanceSensorEx extends Rev2mDistanceSensor {

    long measurement_timing_budget_us;

    protected int io_timeout = 0;
    protected ElapsedTime ioElapsedTime;

    private long lastRead = 0;
    private int rangeCache;

    public Rev2mDistanceSensorEx(I2cDeviceSynch deviceClient, boolean owned) {
        super(deviceClient, owned);
        if(deviceClient instanceof LynxI2cDeviceSynch){
            ((LynxI2cDeviceSynch) deviceClient).setBusSpeed(LynxI2cDeviceSynch.BusSpeed.FAST_400K);
        }
    }

    @Override
    protected int readRangeContinuousMillimeters() {
        if(io_timeout > 0) {
            ioElapsedTime.reset();
        }
        // assumptions: Linearity Corrective Gain is 1000 (default);
        // fractional ranging is not enabled

        if(System.currentTimeMillis() - lastRead > (measurement_timing_budget_us / 1000.0)) {
            byte[] readBytes = deviceClient.read(RESULT_RANGE_STATUS.bVal + 10, 2);
            rangeCache = (int) TypeConversion.byteArrayToShort(readBytes);
            lastRead = System.currentTimeMillis();
        }

        return rangeCache;
    }

    /**
     * Sets ranging profile of the device
     * DEFAULT: 30 hz update, medium accuracy
     * HIGH_SPEED: 50 hz update, low accuracy
     * HIGH_ACCURACY: 10 hz update, high accuracy
     */
    public void setRangingProfile(RANGING_PROFILE rangingProfile){
        switch (rangingProfile){
            case DEFAULT:
                this.setMeasurementTimingBudget(33000);
                break;
            case HIGH_SPEED:
                this.setMeasurementTimingBudget(20000);
                break;
            case HIGH_ACCURACY:
                this.setMeasurementTimingBudget(100000);
                break;
        }
    }

    public enum RANGING_PROFILE{
        DEFAULT,
        HIGH_SPEED,
        HIGH_ACCURACY
    }
}

package com.outoftheboxrobotics.photoncore.hardware;

import android.content.Context;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxVoltageSensor;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.util.LastKnown;

/**
 * Optimized LynxVoltage sensor. Allows the user to get access to the values
 * read by the RC app
 */
public class PhotonLynxVoltageSensor extends LynxVoltageSensor {
    public PhotonLynxVoltageSensor(Context context, LynxModule module) throws RobotCoreException, InterruptedException {
        super(context, module);
        lastKnownVoltage.setValue(0.0);
    }

    private final LastKnown<Double> lastKnownVoltage = new LastKnown<>();

    /**
     * Get the cached voltage. This also gets the voltage that is read by the RC app
     */
    @SuppressWarnings({"unused"})
    public double getCachedVoltage()
    {
        return lastKnownVoltage.getNonTimedValue();
    }
    @Override
    public double getVoltage() {
        double voltage = super.getVoltage();
        lastKnownVoltage.setValue(voltage);
        return voltage;
    }
}

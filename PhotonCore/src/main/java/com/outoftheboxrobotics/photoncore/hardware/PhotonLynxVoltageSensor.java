package com.outoftheboxrobotics.photoncore.hardware;

import android.content.Context;

import com.outoftheboxrobotics.photoncore.Photon;
import com.outoftheboxrobotics.photoncore.PhotonLastKnown;
import com.outoftheboxrobotics.photoncore.hardware.motor.commands.PhotonLynxGetADCCommand;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.hardware.lynx.LynxUnsupportedCommandException;
import com.qualcomm.hardware.lynx.LynxUsbUtil;
import com.qualcomm.hardware.lynx.LynxVoltageSensor;
import com.qualcomm.hardware.lynx.commands.core.LynxGetADCCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetADCResponse;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.util.LastKnown;

import java.util.concurrent.CompletableFuture;

/**
 * Optimized LynxVoltage sensor. Allows the user to get access to the values
 * read by the RC app
 */
public class PhotonLynxVoltageSensor extends LynxVoltageSensor {
    public PhotonLynxVoltageSensor(Context context, LynxModule module) throws RobotCoreException, InterruptedException {
        super(context, module);
        lastKnownVoltage.setValue(0.0);
    }

    // TODO: Check if the interval read by the RC app would be better than setting it as cannot expire
    private final PhotonLastKnown<Double> lastKnownVoltage = new PhotonLastKnown<>(false);

    /**
     * Gets the cached voltage of this voltage sensor. Since it is used by
     * @return
     */
    @SuppressWarnings({"unused"})
    public double getCachedVoltage()
    {
        return lastKnownVoltage.getValue();
    }
    @Override
    public double getVoltage() {
        double voltage = super.getVoltage();
        lastKnownVoltage.setValue(voltage);
        return voltage;
    }

    public CompletableFuture<Double> getVoltageAsync() {
        PhotonLynxGetADCCommand command = new PhotonLynxGetADCCommand(this.getModule(), LynxGetADCCommand.Channel.BATTERY_MONITOR, LynxGetADCCommand.Mode.ENGINEERING);
        try {
            command.acquireNetworkLock();
            getModule().sendCommand(command);
            return command.getResponse().thenApply((message) -> {
                try {
                    command.releaseNetworkLock();
                } catch (InterruptedException e) {
                    handleException(e);
                }
                LynxGetADCResponse response = (LynxGetADCResponse) message;
                int mv = response.getValue();
                return mv * 0.001;
            });
        }
        catch (InterruptedException | RuntimeException | LynxUnsupportedCommandException e)
        {
            handleException(e);
        }
        return CompletableFuture.completedFuture(LynxUsbUtil.makePlaceholderValue(0.0));
    }


}

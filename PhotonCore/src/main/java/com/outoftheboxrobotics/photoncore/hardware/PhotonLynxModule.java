package com.outoftheboxrobotics.photoncore.hardware;

import androidx.annotation.NonNull;

import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.outoftheboxrobotics.photoncore.ReflectionUtils;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxUnsupportedCommandException;
import com.qualcomm.hardware.lynx.LynxUsbDevice;
import com.qualcomm.hardware.lynx.commands.LynxCommand;
import com.qualcomm.hardware.lynx.commands.LynxMessage;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.ArrayList;
import java.util.Objects;

/**
 * Optimized LynxModule, main magic of Photon.
 * Inspired by Photon
 */
@SuppressWarnings({"unused"})
public class PhotonLynxModule extends LynxModule {
    private static final String TAG = "PhotonLynxModule";

    /**
     * Create a PhotonLynxModule from a standard LynxModule
     */
    public static PhotonLynxModule fromLynxModule(LynxModule lynxModule) {
        try {
            PhotonLynxModule module = new PhotonLynxModule(
                    ReflectionUtils.getFieldValue(lynxModule, "lynxUsbDevice"),
                    ReflectionUtils.getFieldValue(lynxModule, "moduleAddress"),
                    ReflectionUtils.getFieldValue(lynxModule, "isParent"),
                    ReflectionUtils.getFieldValue(lynxModule, "isUserModule")
            );
            ReflectionUtils.deepCopy(lynxModule, module);
            return module;
        } catch (Exception e) {

            RobotLog.e(TAG, "fromLynxModule: Failed creating a PhotonLynxModule");
        }
        return null;
    }

    private PhotonLynxModule(LynxUsbDevice lynxUsbDevice, Integer moduleAddress, Boolean isParent, Boolean isUserModule) {
        super(lynxUsbDevice, moduleAddress, isParent, isUserModule);
    }

    public LynxUsbDevice getLynxUsbDevice() {
        return lynxUsbDevice;
    }
    private final ArrayList<LynxCommand<? extends LynxMessage>> inFlightCommands = new ArrayList<>();

    @Override
    public void acquireNetworkTransmissionLock(@NonNull LynxMessage message) throws InterruptedException {
        if(PhotonCore.photon==null) super.acquireNetworkTransmissionLock(message);
    }
    @Override
    public void sendCommand(LynxMessage command) throws InterruptedException, LynxUnsupportedCommandException {
        if(PhotonCore.photon==null) {
            super.sendCommand(command);
            return;
        }
        if(command instanceof LynxCommand)
        {
            synchronized (this)
            {
                while(inFlightCommands.size()>PhotonCore.photon.maximumParallelCommands())
                    inFlightCommands.removeIf(LynxCommand::isAckOrResponseReceived);
                inFlightCommands.add((LynxCommand<? extends LynxMessage>) command);
                super.sendCommand(command);
                return;
            }
        }
        super.sendCommand(command);
    }

    @Override
    public void releaseNetworkTransmissionLock(@NonNull LynxMessage message) throws InterruptedException {
        if(PhotonCore.photon==null)super.releaseNetworkTransmissionLock(message);
    }

    @Override
    public int hashCode() {
        return Objects.hashCode(moduleAddress);
    }

}

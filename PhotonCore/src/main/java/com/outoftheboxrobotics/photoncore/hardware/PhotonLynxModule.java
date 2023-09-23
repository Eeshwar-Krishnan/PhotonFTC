package com.outoftheboxrobotics.photoncore.hardware;

import androidx.annotation.NonNull;

import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.outoftheboxrobotics.photoncore.ReflectionUtils;
import com.outoftheboxrobotics.photoncore.hardware.motor.commands.PhotonLynxSetMotorConstantPowerCommand;
import com.qualcomm.hardware.lynx.LynxController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.hardware.lynx.LynxUnsupportedCommandException;
import com.qualcomm.hardware.lynx.LynxUsbDevice;
import com.qualcomm.hardware.lynx.LynxUsbDeviceImpl;
import com.qualcomm.hardware.lynx.Supplier;
import com.qualcomm.hardware.lynx.commands.LynxCommand;
import com.qualcomm.hardware.lynx.commands.LynxDatagram;
import com.qualcomm.hardware.lynx.commands.LynxInterface;
import com.qualcomm.hardware.lynx.commands.LynxMessage;
import com.qualcomm.hardware.lynx.commands.LynxRespondable;
import com.qualcomm.hardware.lynx.commands.LynxResponse;
import com.qualcomm.hardware.lynx.commands.standard.LynxAck;
import com.qualcomm.hardware.lynx.commands.standard.LynxNack;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.LynxModuleImuType;
import com.qualcomm.robotcore.hardware.usb.RobotArmingStateNotifier;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.SerialNumber;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.TempUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.robotcore.internal.system.Assert;
import org.firstinspires.ftc.robotcore.internal.usb.exception.RobotUsbException;

import java.lang.reflect.InvocationTargetException;
import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.List;
import java.util.Objects;
import java.util.concurrent.atomic.AtomicInteger;

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
                    lynxModule,
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
    private final LynxModule delegate;
    private PhotonLynxModule(LynxModule delegate, LynxUsbDevice lynxUsbDevice, Integer moduleAddress, Boolean isParent, Boolean isUserModule) {
        super(lynxUsbDevice, moduleAddress, isParent, isUserModule);
        this.delegate=delegate;
    }

    public LynxUsbDevice getLynxUsbDevice() {
        return lynxUsbDevice;
    }

    @Override
    public void acquireNetworkTransmissionLock(@NonNull LynxMessage message) throws InterruptedException {
        if(PhotonCore.photon==null) delegate.acquireNetworkTransmissionLock(message);
    }
    @Override
    public void sendCommand(LynxMessage command) throws InterruptedException, LynxUnsupportedCommandException {
        if(PhotonCore.photon==null) {
            delegate.sendCommand(command);
            return;
        }
        if(command instanceof LynxCommand)
        {
            synchronized (this)
            {
                while(unfinishedCommands.size()>PhotonCore.photon.maximumParallelCommands());

                command.setMessageNumber(getNewMessageNumber());
                int msgnumCur = command.getMessageNumber();

                // Serialize this guy and remember it
                LynxDatagram datagram = new LynxDatagram(command); // throws LynxUnsupportedCommandException
                command.setSerialization(datagram);


                this.unfinishedCommands.put(msgnumCur, (LynxRespondable)command);
                // Send it on out!
                this.lynxUsbDevice.transmit(command);
                return;
            }
        }
    }

    @Override
    public void releaseNetworkTransmissionLock(@NonNull LynxMessage message) throws InterruptedException {
        if(PhotonCore.photon==null)delegate.releaseNetworkTransmissionLock(message);
    }

    @Override
    public int hashCode() {
        return Objects.hashCode(moduleAddress);
    }

}

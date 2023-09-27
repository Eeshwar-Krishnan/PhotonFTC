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
    private PhotonLynxModule(LynxModule delegate, LynxUsbDevice lynxUsbDevice, Integer moduleAddress, Boolean isParent, Boolean isUserModule) {
        super(lynxUsbDevice, moduleAddress, isParent, isUserModule);
        stopPingTimer(false);
        delegate.disengage();
    }

    public LynxUsbDevice getLynxUsbDevice() {
        return lynxUsbDevice;
    }

    @Override
    public void acquireNetworkTransmissionLock(@NonNull LynxMessage message) throws InterruptedException {
        if(PhotonCore.photon==null)
            super.acquireNetworkTransmissionLock(message);
    }

    @Override
    public void onIncomingDatagramReceived(LynxDatagram datagram) {
        if(PhotonCore.photon==null) {
            super.onIncomingDatagramReceived(datagram);
            return;
        }
        warnIfClosed();
        noteDatagramReceived();
        // Reify the incoming command. First, what kind of command is that guy?
        try {
            MessageClassAndCtor pair = this.commandClasses.get(datagram.getCommandNumber());
            if (pair != null)
            {
                // Is it the command itself, or a response to a command of that flavor?
                if (datagram.isResponse())
                {
                    pair = responseClasses.get(pair.clazz);
                }
                if (pair != null)
                {
                    // Instantiate the command or response so we can deserialize
                    LynxMessage incomingMessage = pair.ctor.newInstance(this);

                    // Deserialize
                    incomingMessage.setSerialization(datagram);
                    incomingMessage.loadFromSerialization();

                    if (LynxUsbDeviceImpl.DEBUG_LOG_MESSAGES) RobotLog.vv(TAG, "rec'd: mod=%d cmd=0x%02x(%s) msg#=%d ref#=%d", datagram.getSourceModuleAddress(), datagram.getPacketId(), incomingMessage.getClass().getSimpleName(), incomingMessage.getMessageNumber(), incomingMessage.getReferenceNumber());

                    // Acks&nacks are processed differently than responses
                    if (incomingMessage.isAck() || incomingMessage.isNack())
                    {
                        LynxCommand ackdCommand = (LynxCommand) this.unfinishedCommands.get(datagram.getReferenceNumber());
                        if (ackdCommand != null)
                        {
                            // Process the ack or the nack
                            if (incomingMessage.isNack())
                            {
                                ackdCommand.onNackReceived((LynxNack)incomingMessage);
                            }
                            else
                            {
                                ackdCommand.onAckReceived((LynxAck)incomingMessage);
                            }

                            // If we get an ack or a nack, then we WON'T get a response
                            finishedWithMessage(ackdCommand);
                            for (PhotonLynxCommandListener listener:
                                    listeners) {
                                listener.onCommandResponse(incomingMessage, ackdCommand);
                            }
                        }
                        else
                        {
                            RobotLog.ee(TAG, "unable to find originating LynxRespondable for mod=%d msg#=%d ref#=%d", datagram.getSourceModuleAddress(), datagram.getMessageNumber(), datagram.getReferenceNumber());
                        }
                    }
                    else
                    {
                        LynxCommand originatingCommand = (LynxCommand) this.unfinishedCommands.get(datagram.getReferenceNumber());
                        if (originatingCommand != null)
                        {
                            Assert.assertTrue(incomingMessage.isResponse());

                            // Process the response
                            originatingCommand.onResponseReceived((LynxResponse) incomingMessage);
                            for (PhotonLynxCommandListener listener:
                                 listeners) {
                                listener.onCommandResponse((LynxResponse)incomingMessage, originatingCommand);
                            }
                            // After a response is received, we're always done with a command
                            finishedWithMessage(originatingCommand);
                        }
                        else
                        {
                            RobotLog.ee(TAG, "unable to find originating command for packetid=0x%04x msg#=%d ref#=%d", datagram.getPacketId(), datagram.getMessageNumber(), datagram.getReferenceNumber());
                        }
                    }
                }
            }
            else
            {
                RobotLog.ee(TAG, "no command class known for command=0x%02x", datagram.getCommandNumber());
            }
        }
        catch (InstantiationException|IllegalAccessException|InvocationTargetException|RuntimeException e)
        {
            RobotLog.ee(TAG, e, "internal error in LynxModule.noteIncomingDatagramReceived()");
        }

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
                while(unfinishedCommands.size()>PhotonCore.photon.maximumParallelCommands());

                command.setMessageNumber(getNewMessageNumber());
                int msgnumCur = command.getMessageNumber();

                // Serialize this guy and remember it
                LynxDatagram datagram = new LynxDatagram(command); // throws LynxUnsupportedCommandException
                command.setSerialization(datagram);


                this.unfinishedCommands.put(msgnumCur, (LynxRespondable)command);
                // Send it on out!
                this.lynxUsbDevice.transmit(command);
                for (PhotonLynxCommandListener listener:
                        listeners) {
                    listener.onCommand((LynxCommand)command);
                }
            }
        }
    }
    private List<PhotonLynxCommandListener> listeners = new ArrayList<PhotonLynxCommandListener>();
    public boolean addLynxCommandListener(PhotonLynxCommandListener listener)
    {
        listeners.add(listener);
        return true;
    }
    @Override
    public void releaseNetworkTransmissionLock(@NonNull LynxMessage message) throws InterruptedException {
        if(PhotonCore.photon==null)
            super.releaseNetworkTransmissionLock(message);
    }

    @Override
    public int hashCode() {
        return Objects.hashCode(moduleAddress);
    }

}

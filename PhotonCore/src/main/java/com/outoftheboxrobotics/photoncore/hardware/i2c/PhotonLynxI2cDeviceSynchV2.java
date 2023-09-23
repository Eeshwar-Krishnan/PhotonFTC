package com.outoftheboxrobotics.photoncore.hardware.i2c;

import com.outoftheboxrobotics.photoncore.PhotonCommandBase;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.outoftheboxrobotics.photoncore.hardware.PhotonLynxModule;
import com.qualcomm.hardware.lynx.LynxI2cDeviceSynchV2;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.hardware.lynx.Supplier;
import com.qualcomm.hardware.lynx.commands.LynxCommand;
import com.qualcomm.hardware.lynx.commands.LynxMessage;
import com.qualcomm.hardware.lynx.commands.core.LynxI2cWriteReadMultipleBytesCommand;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.I2cWarningManager;
import com.qualcomm.robotcore.hardware.TimestampedData;
import com.qualcomm.robotcore.hardware.TimestampedI2cData;

import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

public class PhotonLynxI2cDeviceSynchV2 extends LynxI2cDeviceSynchV2 implements PhotonI2cDeviceSynch, PhotonInterleavableI2cDevice {
    public PhotonLynxI2cDeviceSynchV2(PhotonLynxModule module, int bus) {
        super(null, module, bus);
    }
    private PhotonCommandBase<? extends LynxMessage> interleavedCommand = null;
    private LynxMessage result;
    @Override
    public void scheduleInterleavedCommand(PhotonCommandBase<? extends LynxMessage> command) {
        interleavedCommand=command;
    }

    @Override
    public LynxMessage getResult() {
        return result;
    }

    @Override
    public synchronized TimestampedData readTimeStamped(int ireg, int creg) {
        if(PhotonCore.photon==null) return super.readTimeStamped(ireg, creg);
        try {
            final Supplier<LynxCommand<?>> readWriteTxSupplier = () -> new LynxI2cWriteReadMultipleBytesCommand(getModule(), bus, i2cAddr, ireg, creg);

            return acquireI2cLockWhile(() -> {
                sendI2cTransaction(readWriteTxSupplier);
                if(PhotonCore.photon!=null&&interleavedCommand!=null)
                {
                    result = interleavedCommand.sendReceive();
                    interleavedCommand=null;
                }
                readTimeStampedPlaceholder.reset();
                return pollForReadResult(i2cAddr, ireg, creg);
            });
        } catch (InterruptedException|RobotCoreException|RuntimeException e) {
            handleException(e);
        } catch (LynxNackException e) {
            /*
             * This is a possible device problem, go ahead and tell I2cWarningManager to warn.
             */
            I2cWarningManager.notifyProblemI2cDevice(this);
            handleException(e);
        }
        return readTimeStampedPlaceholder.log(TimestampedI2cData.makeFakeData(getI2cAddress(), ireg, creg));
    }

}

package com.outoftheboxrobotics.photoncore.hardware.i2c;

import com.outoftheboxrobotics.photoncore.PhotonCommandBase;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.outoftheboxrobotics.photoncore.hardware.PhotonLynxModule;
import com.qualcomm.hardware.lynx.LynxI2cDeviceSynchV1;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.hardware.lynx.Supplier;
import com.qualcomm.hardware.lynx.commands.LynxCommand;
import com.qualcomm.hardware.lynx.commands.LynxMessage;
import com.qualcomm.hardware.lynx.commands.core.LynxI2cReadMultipleBytesCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxI2cReadSingleByteCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxI2cWriteSingleByteCommand;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.I2cWaitControl;
import com.qualcomm.robotcore.hardware.I2cWarningManager;
import com.qualcomm.robotcore.hardware.TimestampedData;
import com.qualcomm.robotcore.hardware.TimestampedI2cData;

import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

public class PhotonLynxI2cDeviceSynchV1 extends LynxI2cDeviceSynchV1 implements PhotonI2cDeviceSynch, PhotonInterleavableI2cDevice {
    public PhotonLynxI2cDeviceSynchV1(PhotonLynxModule module, int bus) {
        super(null, module, bus);
    }
    private PhotonCommandBase<? extends LynxMessage> interleavedCommand = null;
    private LynxMessage result=null;
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
            final Supplier<LynxI2cWriteSingleByteCommand> writeTxSupplier = () -> new LynxI2cWriteSingleByteCommand(getModule(), bus, i2cAddr, ireg);

            final Supplier<LynxCommand<?>> readTxSupplier = () -> {
                /*
                 * LynxI2cReadMultipleBytesCommand does not support a
                 * byte count of one, so we manually differentiate here.
                 */
                return creg==1
                        ? new LynxI2cReadSingleByteCommand(getModule(), bus, i2cAddr)
                        : new LynxI2cReadMultipleBytesCommand(getModule(), bus, i2cAddr, creg);
            };

            return acquireI2cLockWhile(() -> {
                sendI2cTransaction(writeTxSupplier);
                internalWaitForWriteCompletions(I2cWaitControl.ATOMIC);
                sendI2cTransaction(readTxSupplier);
                if(PhotonCore.photon!=null&&interleavedCommand!=null)
                {
                    result=interleavedCommand.sendReceive();
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

package com.outoftheboxrobotics.photoncore.HAL.I2C;

import android.content.Context;

import com.outoftheboxrobotics.photoncore.HAL.HAL;
import com.outoftheboxrobotics.photoncore.HAL.I2C.Commands.PhotonLynxI2cReadMultipleBytesCommand;
import com.outoftheboxrobotics.photoncore.HAL.I2C.Commands.PhotonLynxI2cReadSingleByteCommand;
import com.outoftheboxrobotics.photoncore.HAL.I2C.Commands.PhotonLynxI2cWriteSingleByteCommand;
import com.qualcomm.hardware.lynx.LynxI2cDeviceSynchV1;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.hardware.lynx.Supplier;
import com.qualcomm.hardware.lynx.commands.LynxCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxI2cWriteSingleByteCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxI2cWriteStatusQueryCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxI2cWriteStatusQueryResponse;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cWaitControl;
import com.qualcomm.robotcore.hardware.I2cWarningManager;
import com.qualcomm.robotcore.hardware.TimestampedData;
import com.qualcomm.robotcore.hardware.TimestampedI2cData;

public class PhotonLynxI2cDeviceSynchV1 extends LynxI2cDeviceSynchV1 implements PhotonLynxI2cDeviceSynch {
    private HAL hal;
    private LynxCommand<?> interleavedCommand;
    public PhotonLynxI2cDeviceSynchV1(HAL hal, Context context, LynxModule module, int bus) {
        super(context, module, bus);
        this.hal=hal;
    }
    public void setInterleavedCommand(LynxCommand<?> command)
    {
        this.interleavedCommand=command;
    }
    public void clearInterleavedCommand()
    {
        this.interleavedCommand=null;
    }
    @Override
    public synchronized TimestampedData readTimeStamped(int ireg, int creg) {
        try {
            final Supplier<LynxI2cWriteSingleByteCommand> writeTxSupplier = new Supplier<LynxI2cWriteSingleByteCommand>()
            {
                @Override
                public LynxI2cWriteSingleByteCommand get()
                {
                    return new PhotonLynxI2cWriteSingleByteCommand(getModule(), bus, i2cAddr, ireg);
                }
            };

            final Supplier<LynxCommand<?>> readTxSupplier = new Supplier<LynxCommand<?>>()
            {
                @Override
                public LynxCommand<?> get()
                {
                    /*
                     * LynxI2cReadMultipleBytesCommand does not support a
                     * byte count of one, so we manually differentiate here.
                     */
                    return creg==1
                            ? new PhotonLynxI2cReadSingleByteCommand(getModule(), bus, i2cAddr)
                            : new PhotonLynxI2cReadMultipleBytesCommand(getModule(), bus, i2cAddr, creg);
                }
            };

            return acquireI2cLockWhile(new Supplier<TimestampedData>()
            {
                @Override public TimestampedData get() throws InterruptedException, RobotCoreException, LynxNackException
                {
                    sendI2cTransaction(writeTxSupplier);
                    internalWaitForWriteCompletions(I2cWaitControl.ATOMIC);
                    sendI2cTransaction(readTxSupplier);
                    if(interleavedCommand!=null)
                        hal.write(interleavedCommand);
                    readTimeStampedPlaceholder.reset();
                    return pollForReadResult(i2cAddr, ireg, creg);
                }
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
    @Override
    protected void sendI2cTransaction(Supplier<? extends LynxCommand<?>> transactionSupplier) throws LynxNackException, InterruptedException, RobotCoreException
    {
        for (;;)
        {
            try {
                hal.write(transactionSupplier.get());
                break;
            }
            catch (LynxNackException e)
            {
                switch (e.getNack().getNackReasonCodeAsEnum())
                {
                    case I2C_MASTER_BUSY:
                    case I2C_OPERATION_IN_PROGRESS:
                        break;
                    default:
                        throw e;
                }
            }
        }
    }
    // TODO: override internalWaitForWriteCompletions and pollForResults when hal has sendReceive
}

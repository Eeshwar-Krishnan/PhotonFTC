package com.outoftheboxrobotics.photoncore.hardware.i2c.commands;

import com.outoftheboxrobotics.photoncore.PhotonCommandBase;
import com.qualcomm.hardware.lynx.LynxModuleIntf;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.hardware.lynx.commands.LynxInterface;
import com.qualcomm.hardware.lynx.commands.LynxMessage;
import com.qualcomm.hardware.lynx.commands.core.LynxI2cReadMultipleBytesCommand;
import com.qualcomm.hardware.lynx.commands.standard.LynxAck;
import com.qualcomm.hardware.lynx.commands.standard.LynxNack;
import com.qualcomm.robotcore.hardware.I2cAddr;

import java.util.concurrent.CompletableFuture;

public class PhotonLynxI2cReadMultipleBytesCommand extends LynxI2cReadMultipleBytesCommand implements PhotonCommandBase {
    private CompletableFuture<LynxMessage> future;
    public PhotonLynxI2cReadMultipleBytesCommand(LynxModuleIntf module) {
        super(module);
        future=new CompletableFuture<>();
    }

    public PhotonLynxI2cReadMultipleBytesCommand(LynxModuleIntf module, int busZ, I2cAddr i2cAddr, int cbToRead) {
        super(module, busZ, i2cAddr, cbToRead);
        future=new CompletableFuture<>();
    }
    @Override
    public void onResponseReceived(LynxMessage response) {
        super.onResponseReceived(response);
        future.complete(response);
    }

    @Override
    public void onAckReceived(LynxAck ack) {
        super.onAckReceived(ack);
        future.complete(ack);
    }

    @Override
    public void onNackReceived(LynxNack nack) {
        super.onNackReceived(nack);
        future.complete(nack);
    }

    @Override
    public CompletableFuture<LynxMessage> getResponse() {
        return future;
    }
    @Override
    public int getCommandNumber() {
        LynxInterface theInterface = this.getInterface();
        if (null == theInterface)
            return LynxInterface.ERRONEOUS_COMMAND_NUMBER;   // should never happen in working system, but might if pretending

        return theInterface.getBaseCommandNumber() + 40;
    }
}

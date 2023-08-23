package com.outoftheboxrobotics.photoncore.hardware.i2c.commands;

import com.outoftheboxrobotics.photoncore.PhotonCommandBase;
import com.qualcomm.hardware.lynx.LynxModuleIntf;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.hardware.lynx.commands.LynxMessage;
import com.qualcomm.hardware.lynx.commands.core.LynxI2cWriteReadMultipleBytesCommand;
import com.qualcomm.hardware.lynx.commands.standard.LynxAck;
import com.qualcomm.hardware.lynx.commands.standard.LynxNack;
import com.qualcomm.robotcore.hardware.I2cAddr;

import java.util.concurrent.CompletableFuture;

public class PhotonLynxI2cWriteReadMultipleBytesCommand extends LynxI2cWriteReadMultipleBytesCommand implements PhotonCommandBase {
    private CompletableFuture<LynxMessage> future;
    public PhotonLynxI2cWriteReadMultipleBytesCommand(LynxModuleIntf module) {
        super(module);
        future=new CompletableFuture<>();
    }

    public PhotonLynxI2cWriteReadMultipleBytesCommand(LynxModuleIntf module, int busZ, I2cAddr i2cAddr, int i2cStartAddr, int cbToRead) {
        super(module, busZ, i2cAddr, i2cStartAddr, cbToRead);
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
    public CompletableFuture<LynxMessage> getResponse() throws LynxNackException {
        return future;
    }
}

package com.outoftheboxrobotics.photoncore.hardware.motor.commands;

import com.outoftheboxrobotics.photoncore.PhotonCommandBase;
import com.qualcomm.hardware.lynx.LynxModuleIntf;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.hardware.lynx.commands.LynxMessage;
import com.qualcomm.hardware.lynx.commands.core.LynxGetADCCommand;
import com.qualcomm.hardware.lynx.commands.standard.LynxAck;
import com.qualcomm.hardware.lynx.commands.standard.LynxNack;

import java.util.concurrent.CompletableFuture;

public class PhotonLynxGetADCCommand extends LynxGetADCCommand implements PhotonCommandBase {

    private CompletableFuture<LynxMessage> future;

    public PhotonLynxGetADCCommand(LynxModuleIntf module, Channel channel, Mode mode) {
        super(module, channel, mode);
    }

    @Override
    public void onResponseReceived(LynxMessage response) {
        future.complete(response);
        super.onResponseReceived(response);
    }

    @Override
    public void onAckReceived(LynxAck ack) {
        future.complete(ack);
        super.onAckReceived(ack);
    }

    @Override
    public void onNackReceived(LynxNack nack) {
        future.complete(nack);
        super.onNackReceived(nack);
    }

    @Override
    public CompletableFuture<LynxMessage> getResponse() throws LynxNackException {
        return future;
    }

    @Override
    public void acquireNetworkLock() throws InterruptedException {
        return;
    }

    @Override
    public void releaseNetworkLock() throws InterruptedException {
        return;
    }
}

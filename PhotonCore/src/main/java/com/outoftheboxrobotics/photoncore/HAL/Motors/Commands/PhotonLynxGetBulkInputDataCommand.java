package com.outoftheboxrobotics.photoncore.HAL.Motors.Commands;

import com.qualcomm.hardware.lynx.LynxModuleIntf;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.hardware.lynx.commands.LynxMessage;
import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataCommand;
import com.qualcomm.hardware.lynx.commands.standard.LynxAck;
import com.qualcomm.hardware.lynx.commands.standard.LynxNack;

import java.util.concurrent.CompletableFuture;

public class PhotonLynxGetBulkInputDataCommand extends LynxGetBulkInputDataCommand implements PhotonCommandBase {
    private final CompletableFuture<LynxMessage> future = new CompletableFuture<>();

    public PhotonLynxGetBulkInputDataCommand(LynxModuleIntf module) {
        super(module);
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

package com.outoftheboxrobotics.photoncore.hardware.motor.commands;

import com.outoftheboxrobotics.photoncore.PhotonCommandBase;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModuleIntf;
import com.qualcomm.hardware.lynx.commands.LynxInterface;
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
    public void acquireNetworkLock() throws InterruptedException {
        if(!PhotonCore.attached()){
            super.acquireNetworkLock();
        }else {
            PhotonCore.acquire(module);
        }
    }

    @Override
    public void releaseNetworkLock() throws InterruptedException {
        if(!PhotonCore.attached()){
            super.releaseNetworkLock();
        }else {
            PhotonCore.release(module);
        }
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

        return theInterface.getBaseCommandNumber() + 0;
    }
}

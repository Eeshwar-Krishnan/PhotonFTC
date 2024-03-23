package com.outoftheboxrobotics.photoncore.hardware.motor.commands;

import com.outoftheboxrobotics.photoncore.PhotonCommandBase;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModuleIntf;
import com.qualcomm.hardware.lynx.commands.LynxInterface;
import com.qualcomm.hardware.lynx.commands.LynxMessage;
import com.qualcomm.hardware.lynx.commands.core.LynxResetMotorEncoderCommand;
import com.qualcomm.hardware.lynx.commands.standard.LynxAck;
import com.qualcomm.hardware.lynx.commands.standard.LynxNack;

import java.util.concurrent.CompletableFuture;

public class PhotonLynxResetMotorEncoderCommand extends LynxResetMotorEncoderCommand implements PhotonCommandBase {
    private final CompletableFuture<LynxMessage> future = new CompletableFuture<>();

    public PhotonLynxResetMotorEncoderCommand(LynxModuleIntf module, int motorZ) {
        super(module, motorZ);
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

        return theInterface.getBaseCommandNumber() + 14;
    }
}

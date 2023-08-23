package com.outoftheboxrobotics.photoncore.hardware.motor.commands;

import com.outoftheboxrobotics.photoncore.PhotonCommandBase;
import com.qualcomm.hardware.lynx.LynxModuleIntf;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.hardware.lynx.commands.LynxMessage;
import com.qualcomm.hardware.lynx.commands.core.LynxSetMotorTargetPositionCommand;
import com.qualcomm.hardware.lynx.commands.standard.LynxAck;
import com.qualcomm.hardware.lynx.commands.standard.LynxNack;

import java.util.concurrent.CompletableFuture;

public class PhotonLynxSetMotorTargetPositionCommand extends LynxSetMotorTargetPositionCommand implements PhotonCommandBase {
    private final CompletableFuture<LynxMessage> future = new CompletableFuture<>();

    public PhotonLynxSetMotorTargetPositionCommand(LynxModuleIntf module, int motorZ, int target, int tolerance) {
        super(module, motorZ, target, tolerance);
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

    @Override
    public void acquireNetworkLock() throws InterruptedException {
        return;
    }

    @Override
    public void releaseNetworkLock() throws InterruptedException {
        return;
    }
}

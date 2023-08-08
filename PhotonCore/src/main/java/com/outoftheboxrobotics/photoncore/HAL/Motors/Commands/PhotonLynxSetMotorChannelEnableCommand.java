package com.outoftheboxrobotics.photoncore.HAL.Motors.Commands;

import com.qualcomm.hardware.lynx.LynxModuleIntf;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.hardware.lynx.commands.LynxDatagram;
import com.qualcomm.hardware.lynx.commands.LynxMessage;
import com.qualcomm.hardware.lynx.commands.LynxResponse;
import com.qualcomm.hardware.lynx.commands.core.LynxSetMotorChannelEnableCommand;
import com.qualcomm.hardware.lynx.commands.standard.LynxAck;
import com.qualcomm.hardware.lynx.commands.standard.LynxNack;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;

import java.nio.ByteBuffer;
import java.util.concurrent.CompletableFuture;

public class PhotonLynxSetMotorChannelEnableCommand extends LynxSetMotorChannelEnableCommand implements PhotonCommandBase{
    private final CompletableFuture<LynxMessage> future = new CompletableFuture<>();

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
        return;
    }

    @Override
    public void releaseNetworkLock() throws InterruptedException {
        return;
    }

    @Override
    public CompletableFuture<LynxMessage> getResponse() throws LynxNackException {
        return future;
    }

    public PhotonLynxSetMotorChannelEnableCommand(LynxModuleIntf module, int motorz, boolean enabled) {
        super(module, motorz, enabled);
    }
}

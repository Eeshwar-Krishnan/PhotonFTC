package com.outoftheboxrobotics.photoncore.HAL.Motors.Commands;

import com.qualcomm.hardware.lynx.LynxModuleIntf;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.hardware.lynx.commands.LynxMessage;
import com.qualcomm.hardware.lynx.commands.core.LynxSetMotorChannelEnableCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxSetMotorTargetVelocityCommand;
import com.qualcomm.hardware.lynx.commands.standard.LynxAck;
import com.qualcomm.hardware.lynx.commands.standard.LynxNack;

import java.util.concurrent.CompletableFuture;

public class PhotonLynxSetMotorTargetVelocityCommand extends LynxSetMotorTargetVelocityCommand implements PhotonCommandBase{
    private CompletableFuture<LynxMessage> future;

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

    public PhotonLynxSetMotorTargetVelocityCommand(LynxModuleIntf module, int motorZ, int velocity) {
        super(module, motorZ, velocity);
    }

    @Override
    public CompletableFuture<LynxMessage> getResponse() throws LynxNackException {
        return null;
    }
}

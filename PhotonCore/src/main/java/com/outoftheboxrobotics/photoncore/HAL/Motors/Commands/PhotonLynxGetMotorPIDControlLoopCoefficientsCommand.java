package com.outoftheboxrobotics.photoncore.HAL.Motors.Commands;

import com.qualcomm.hardware.lynx.LynxModuleIntf;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.hardware.lynx.commands.LynxMessage;
import com.qualcomm.hardware.lynx.commands.core.LynxGetMotorPIDControlLoopCoefficientsCommand;
import com.qualcomm.hardware.lynx.commands.standard.LynxAck;
import com.qualcomm.hardware.lynx.commands.standard.LynxNack;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.concurrent.CompletableFuture;

public class PhotonLynxGetMotorPIDControlLoopCoefficientsCommand extends LynxGetMotorPIDControlLoopCoefficientsCommand implements PhotonCommandBase {
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

    @Override
    public void acquireNetworkLock() throws InterruptedException {
        //Nah, I'm gonna do my own thing
    }

    @Override
    public void releaseNetworkLock() throws InterruptedException {
        //Nah, I'm gonna do my own thing
    }

    public PhotonLynxGetMotorPIDControlLoopCoefficientsCommand(LynxModuleIntf module, int motorZ, DcMotor.RunMode mode) {
        super(module, motorZ, mode);
    }

    @Override
    public CompletableFuture<LynxMessage> getResponse() throws LynxNackException {
        return future;
    }
}

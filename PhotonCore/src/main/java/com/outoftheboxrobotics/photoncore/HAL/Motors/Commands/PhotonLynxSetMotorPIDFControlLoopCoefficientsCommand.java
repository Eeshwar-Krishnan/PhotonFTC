package com.outoftheboxrobotics.photoncore.HAL.Motors.Commands;

import com.qualcomm.hardware.lynx.LynxModuleIntf;
import com.qualcomm.hardware.lynx.commands.LynxMessage;
import com.qualcomm.hardware.lynx.commands.core.LynxSetMotorPIDFControlLoopCoefficientsCommand;
import com.qualcomm.hardware.lynx.commands.standard.LynxAck;
import com.qualcomm.hardware.lynx.commands.standard.LynxNack;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.concurrent.CompletableFuture;

public class PhotonLynxSetMotorPIDFControlLoopCoefficientsCommand extends LynxSetMotorPIDFControlLoopCoefficientsCommand {
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

    public PhotonLynxSetMotorPIDFControlLoopCoefficientsCommand(LynxModuleIntf module, int motorZ, DcMotor.RunMode mode, int p, int i, int d, int f, InternalMotorControlAlgorithm motorControlAlgorithm) {
        super(module, motorZ, mode, p, i, d, f, motorControlAlgorithm);
    }
}

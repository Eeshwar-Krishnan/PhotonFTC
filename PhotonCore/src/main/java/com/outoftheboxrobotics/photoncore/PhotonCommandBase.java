package com.outoftheboxrobotics.photoncore;

import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.hardware.lynx.commands.LynxMessage;

import java.util.concurrent.CompletableFuture;

public interface PhotonCommandBase<RESPONSE extends LynxMessage>{
    CompletableFuture<LynxMessage> getResponse();
    void send() throws InterruptedException, LynxNackException;
    RESPONSE sendReceive() throws InterruptedException, LynxNackException;
}

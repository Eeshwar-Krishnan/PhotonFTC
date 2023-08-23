package com.outoftheboxrobotics.photoncore.hardware.i2c;

import com.outoftheboxrobotics.photoncore.PhotonCommandBase;
import com.qualcomm.hardware.lynx.commands.LynxMessage;

import java.util.concurrent.CompletableFuture;

public interface PhotonInterleavableI2cDevice {
    void scheduleInterleavedCommand(PhotonCommandBase<? extends LynxMessage> command);
    CompletableFuture<? extends LynxMessage> getResponse();
}

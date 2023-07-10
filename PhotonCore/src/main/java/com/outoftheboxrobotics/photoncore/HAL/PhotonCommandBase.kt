package com.outoftheboxrobotics.photoncore.HAL

import com.qualcomm.hardware.lynx.LynxNackException
import com.qualcomm.hardware.lynx.commands.LynxMessage
import java.util.concurrent.CompletableFuture

interface PhotonCommandBase {
    fun getResponse(): CompletableFuture<LynxMessage>
}
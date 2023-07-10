package com.outoftheboxrobotics.photoncore.HAL.I2C.Commands

import com.outoftheboxrobotics.photoncore.HAL.Motors.Commands.PhotonCommandBase
import com.qualcomm.hardware.lynx.LynxModuleIntf
import com.qualcomm.hardware.lynx.LynxNackException
import com.qualcomm.hardware.lynx.commands.LynxMessage
import com.qualcomm.hardware.lynx.commands.core.LynxI2cWriteStatusQueryCommand
import com.qualcomm.hardware.lynx.commands.standard.LynxAck
import com.qualcomm.hardware.lynx.commands.standard.LynxNack
import java.util.concurrent.CompletableFuture

class PhotonLynxI2cWriteStatusQueryCommand : LynxI2cWriteStatusQueryCommand, PhotonCommandBase {
    private val future: CompletableFuture<LynxMessage> = CompletableFuture();

    constructor(module: LynxModuleIntf?) : super(module)
    constructor(module: LynxModuleIntf?, busZ: Int) : super(module, busZ)

    override fun onResponseReceived(response: LynxMessage) {
        super.onResponseReceived(response)
        future.complete(response)
    }

    override fun onAckReceived(ack: LynxAck) {
        super.onAckReceived(ack)
        future.complete(ack)
    }

    override fun onNackReceived(nack: LynxNack) {
        super.onNackReceived(nack)
        future.complete(nack)
    }

    override fun getResponse(): CompletableFuture<LynxMessage> {
        return future
    }
}
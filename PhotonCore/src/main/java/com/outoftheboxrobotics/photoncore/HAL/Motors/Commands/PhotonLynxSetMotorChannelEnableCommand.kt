package com.outoftheboxrobotics.photoncore.HAL.Motors.Commands

import com.outoftheboxrobotics.photoncore.HAL.PhotonCommandBase
import com.qualcomm.hardware.lynx.LynxModuleIntf
import com.qualcomm.hardware.lynx.LynxNackException
import com.qualcomm.hardware.lynx.commands.LynxMessage
import com.qualcomm.hardware.lynx.commands.core.LynxSetMotorChannelEnableCommand
import com.qualcomm.hardware.lynx.commands.standard.LynxAck
import com.qualcomm.hardware.lynx.commands.standard.LynxNack
import java.util.concurrent.CompletableFuture

class PhotonLynxSetMotorChannelEnableCommand(module: LynxModuleIntf?, motorz: Int, enabled: Boolean) : LynxSetMotorChannelEnableCommand(module, motorz, enabled), PhotonCommandBase {
    val future: CompletableFuture<LynxMessage> = CompletableFuture()

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
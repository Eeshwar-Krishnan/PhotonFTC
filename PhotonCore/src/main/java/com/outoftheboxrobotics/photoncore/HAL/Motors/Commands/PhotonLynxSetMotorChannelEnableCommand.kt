package com.outoftheboxrobotics.photoncore.HAL.Motors.Commands

import com.qualcomm.hardware.lynx.LynxModuleIntf
import com.qualcomm.hardware.lynx.LynxNackException
import com.qualcomm.hardware.lynx.commands.LynxMessage
import com.qualcomm.hardware.lynx.commands.core.LynxSetMotorChannelEnableCommand
import com.qualcomm.hardware.lynx.commands.standard.LynxAck
import com.qualcomm.hardware.lynx.commands.standard.LynxNack
import java.util.concurrent.CompletableFuture

class PhotonLynxSetMotorChannelEnableCommand(module: LynxModuleIntf?, motorz: Int, enabled: Boolean) :
    LynxSetMotorChannelEnableCommand(module, motorz, enabled), PhotonCommandBase {
    @get:Throws(LynxNackException::class)
    override val response: CompletableFuture<LynxMessage?>? = null

    override fun onResponseReceived(response: LynxMessage) {
        super.onResponseReceived(response)
        response.complete(response)
    }

    override fun onAckReceived(ack: LynxAck) {
        super.onAckReceived(ack)
        response!!.complete(ack)
    }

    override fun onNackReceived(nack: LynxNack) {
        super.onNackReceived(nack)
        response!!.complete(nack)
    }
}
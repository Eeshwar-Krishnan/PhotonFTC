package com.outoftheboxrobotics.photoncore.HAL.Motors.Commands

import com.qualcomm.hardware.lynx.LynxModuleIntf
import com.qualcomm.hardware.lynx.LynxNackException
import com.qualcomm.hardware.lynx.commands.LynxMessage
import com.qualcomm.hardware.lynx.commands.core.LynxGetMotorChannelEnableCommand
import com.qualcomm.hardware.lynx.commands.standard.LynxAck
import com.qualcomm.hardware.lynx.commands.standard.LynxNack
import java.util.concurrent.CompletableFuture


class PhotonLynxGetMotorChannelEnableCommand<RESPONSE : LynxMessage?>(module: LynxModuleIntf?, motorz: Int, override val response: CompletableFuture<LynxMessage?>?) : LynxGetMotorChannelEnableCommand(module, motorz), PhotonCommandBase {

    private val future: CompletableFuture<LynxMessage>? = null;

    override fun onResponseReceived(response: LynxMessage) {
        future?.complete(response)
        super.onResponseReceived(response)
    }

    override fun onAckReceived(ack: LynxAck) {
        response!!.complete(ack)
        super.onAckReceived(ack)
    }

    override fun onNackReceived(nack: LynxNack) {
        response!!.complete(nack)
        super.onNackReceived(nack)
    }
}
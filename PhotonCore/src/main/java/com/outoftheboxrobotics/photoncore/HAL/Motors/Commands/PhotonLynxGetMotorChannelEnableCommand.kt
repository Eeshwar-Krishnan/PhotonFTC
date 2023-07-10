package com.outoftheboxrobotics.photoncore.HAL.Motors.Commands

import com.outoftheboxrobotics.photoncore.HAL.PhotonCommandBase
import com.qualcomm.hardware.lynx.LynxModuleIntf
import com.qualcomm.hardware.lynx.commands.LynxMessage
import com.qualcomm.hardware.lynx.commands.core.LynxGetMotorChannelEnableCommand
import com.qualcomm.hardware.lynx.commands.standard.LynxAck
import com.qualcomm.hardware.lynx.commands.standard.LynxNack
import com.qualcomm.robotcore.hardware.I2cAddr
import java.util.concurrent.CompletableFuture


class PhotonLynxGetMotorChannelEnableCommand<T>(module: LynxModuleIntf?, motorz: Int) : LynxGetMotorChannelEnableCommand(module, motorz), PhotonCommandBase {

    private val future: CompletableFuture<LynxMessage> = CompletableFuture()

    override fun onResponseReceived(response: LynxMessage) {
        future.complete(response)
        super.onResponseReceived(response)
    }

    override fun onAckReceived(ack: LynxAck) {
        future.complete(ack)
        super.onAckReceived(ack)
    }

    override fun onNackReceived(nack: LynxNack) {
        future.complete(nack)
        super.onNackReceived(nack)
    }

    override fun getResponse(): CompletableFuture<LynxMessage> {
        return future
    }


}
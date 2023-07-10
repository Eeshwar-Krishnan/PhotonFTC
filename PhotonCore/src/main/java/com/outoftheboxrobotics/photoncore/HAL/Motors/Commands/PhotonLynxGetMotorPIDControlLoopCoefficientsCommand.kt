package com.outoftheboxrobotics.photoncore.HAL.Motors.Commands

import com.qualcomm.hardware.lynx.LynxModuleIntf
import com.qualcomm.hardware.lynx.LynxNackException
import com.qualcomm.hardware.lynx.commands.LynxMessage
import com.qualcomm.hardware.lynx.commands.core.LynxGetMotorPIDControlLoopCoefficientsCommand
import com.qualcomm.hardware.lynx.commands.standard.LynxAck
import com.qualcomm.hardware.lynx.commands.standard.LynxNack
import com.qualcomm.robotcore.hardware.DcMotor.RunMode
import java.util.concurrent.CompletableFuture

class PhotonLynxGetMotorPIDControlLoopCoefficientsCommand(module: LynxModuleIntf?, motorZ: Int, mode: RunMode?) :
    LynxGetMotorPIDControlLoopCoefficientsCommand(module, motorZ, mode), PhotonCommandBase {
    private val future: CompletableFuture<LynxMessage>? = null
    override fun onResponseReceived(response: LynxMessage) {
        super.onResponseReceived(response)
        future!!.complete(response)
    }

    override fun onAckReceived(ack: LynxAck) {
        super.onAckReceived(ack)
        future!!.complete(ack)
    }

    override fun onNackReceived(nack: LynxNack) {
        super.onNackReceived(nack)
        future!!.complete(nack)
    }

    @get:Throws(LynxNackException::class)
    override val response: CompletableFuture<LynxMessage?>?
        get() = null
}
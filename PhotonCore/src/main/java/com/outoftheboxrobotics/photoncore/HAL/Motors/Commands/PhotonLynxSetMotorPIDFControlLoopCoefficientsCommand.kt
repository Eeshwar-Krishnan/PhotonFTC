package com.outoftheboxrobotics.photoncore.HAL.Motors.Commands

import com.qualcomm.hardware.lynx.LynxModuleIntf
import com.qualcomm.hardware.lynx.commands.LynxMessage
import com.qualcomm.hardware.lynx.commands.core.LynxSetMotorPIDFControlLoopCoefficientsCommand
import com.qualcomm.hardware.lynx.commands.standard.LynxAck
import com.qualcomm.hardware.lynx.commands.standard.LynxNack
import com.qualcomm.robotcore.hardware.DcMotor.RunMode
import java.util.concurrent.CompletableFuture

class PhotonLynxSetMotorPIDFControlLoopCoefficientsCommand(module: LynxModuleIntf?, motorZ: Int, mode: RunMode?, p: Int, i: Int, d: Int, f: Int, motorControlAlgorithm: InternalMotorControlAlgorithm?) : LynxSetMotorPIDFControlLoopCoefficientsCommand(module, motorZ, mode, p, i, d, f, motorControlAlgorithm) {
    private val future: CompletableFuture<LynxMessage> = CompletableFuture()
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


}
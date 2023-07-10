package com.outoftheboxrobotics.photoncore.HAL.I2C.Commands

import com.outoftheboxrobotics.photoncore.HAL.Motors.Commands.PhotonCommandBase
import com.qualcomm.hardware.lynx.LynxModuleIntf
import com.qualcomm.hardware.lynx.LynxNackException
import com.qualcomm.hardware.lynx.commands.LynxMessage
import com.qualcomm.hardware.lynx.commands.core.LynxI2cWriteSingleByteCommand
import com.qualcomm.hardware.lynx.commands.standard.LynxAck
import com.qualcomm.hardware.lynx.commands.standard.LynxNack
import com.qualcomm.robotcore.hardware.I2cAddr
import java.util.concurrent.CompletableFuture

class PhotonLynxI2cWriteSingleByteCommand : LynxI2cWriteSingleByteCommand, PhotonCommandBase {
    private val future: CompletableFuture<LynxMessage> = CompletableFuture();

    constructor(module: LynxModuleIntf?) : super(module)

    constructor(module: LynxModuleIntf?, busZ: Int, i2cAddr: I2cAddr?, bValue: Int) : super(module, busZ, i2cAddr, bValue)

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
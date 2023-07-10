package com.outoftheboxrobotics.photoncore.HAL.I2C.Commands

import com.outoftheboxrobotics.photoncore.HAL.PhotonCommandBase
import com.qualcomm.hardware.lynx.LynxModuleIntf
import com.qualcomm.hardware.lynx.commands.LynxMessage
import com.qualcomm.hardware.lynx.commands.core.LynxI2cWriteReadMultipleBytesCommand
import com.qualcomm.hardware.lynx.commands.standard.LynxAck
import com.qualcomm.hardware.lynx.commands.standard.LynxNack
import com.qualcomm.robotcore.hardware.I2cAddr
import java.util.concurrent.CompletableFuture

class PhotonLynxI2cWriteReadMultipleBytesCommand : LynxI2cWriteReadMultipleBytesCommand, PhotonCommandBase {
    private val future: CompletableFuture<LynxMessage> = CompletableFuture();

    constructor(module: LynxModuleIntf?) : super(module)

    constructor(module: LynxModuleIntf?, busZ: Int, i2cAddr: I2cAddr?, i2cStartAddr: Int, cbToRead: Int) : super(module, busZ, i2cAddr, i2cStartAddr, cbToRead)

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
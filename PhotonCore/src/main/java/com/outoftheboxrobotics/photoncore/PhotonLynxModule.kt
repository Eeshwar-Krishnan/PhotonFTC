package com.outoftheboxrobotics.photoncore

import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.hardware.lynx.LynxUnsupportedCommandException
import com.qualcomm.hardware.lynx.LynxUsbDevice
import com.qualcomm.hardware.lynx.commands.LynxCommand
import com.qualcomm.hardware.lynx.commands.LynxMessage
import com.qualcomm.hardware.lynx.commands.LynxRespondable
import java.util.concurrent.ConcurrentHashMap

class PhotonLynxModule(lynxUsbDevice: LynxUsbDevice?, moduleAddress: Int, isParent: Boolean, isUserModule: Boolean) :
    LynxModule(lynxUsbDevice, moduleAddress, isParent, isUserModule) {
    private val skippedAcquire = ArrayList<LynxMessage>()
    val unfinishedCommands: ConcurrentHashMap<Int, LynxRespondable<*>>
        get() = this.unfinishedCommands

    @Throws(InterruptedException::class, LynxUnsupportedCommandException::class)
    override fun sendCommand(command: LynxMessage) {
        if (!PhotonCore.instance.enabled.get()) {
            super.sendCommand(command)
            return
        }
        if (command is LynxCommand<*>) {
            if (PhotonCore.getCacheResponse(command) != null) {
                command.onResponseReceived(PhotonCore.getCacheResponse(command))
                return
            }
            if (PhotonCore.shouldParallelize(command)) {
                val success = PhotonCore.registerSend(command)
                if (!success) { super.sendCommand(command) }
                return
            }
        }
        super.sendCommand(command)
    }

    @Throws(InterruptedException::class)
    override fun acquireNetworkTransmissionLock(message: LynxMessage) {
        if (!PhotonCore.instance.enabled.get()) {
            super.acquireNetworkTransmissionLock(message)
            return
        }
        if (message is LynxCommand<*>) {
            if (PhotonCore.getCacheResponse(message) != null) {
                skippedAcquire.add(message)
                return
            }
            if (PhotonCore.shouldParallelize(message)) {
                skippedAcquire.add(message)
                return
            }
        }
        super.acquireNetworkTransmissionLock(message)
    }

    @Throws(InterruptedException::class)
    override fun releaseNetworkTransmissionLock(message: LynxMessage) {
        if (!PhotonCore.instance.enabled.get()) {
            super.releaseNetworkTransmissionLock(message)
            return
        }
        if (skippedAcquire.contains(message)) {
            skippedAcquire.remove(message)
            return
        }
        super.releaseNetworkTransmissionLock(message)
    }
}

package com.outoftheboxrobotics.photoncore.HAL

import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.hardware.lynx.LynxUnsupportedCommandException
import com.qualcomm.hardware.lynx.commands.LynxRespondable
import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataResponse
import java.util.*


class PhotonHAL(override val lynxModule: LynxModule) : HAL {
    private val theLock = Any()
    private val numbersQueue = LinkedList<Int>()

    init {
        for (b in 0..254) {
            numbersQueue.add(b + 1)
        }
    }

    override fun write(respondable: LynxRespondable<*>?) {
        synchronized(theLock) {
            try {
                lynxModule.sendCommand(respondable)
            } catch (e: InterruptedException) {
                e.printStackTrace()
            } catch (e: LynxUnsupportedCommandException) {
                e.printStackTrace()
            }
        }
    }

    override val bulkData: LynxGetBulkInputDataResponse?
        get() = null
}
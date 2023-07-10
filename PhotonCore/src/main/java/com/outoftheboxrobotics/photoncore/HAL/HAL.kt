package com.outoftheboxrobotics.photoncore.HAL

import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.hardware.lynx.commands.LynxRespondable
import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataResponse

interface HAL {
    fun write(respondable: LynxRespondable<*>?)
    val lynxModule: LynxModule?
    val bulkData: LynxGetBulkInputDataResponse?
}
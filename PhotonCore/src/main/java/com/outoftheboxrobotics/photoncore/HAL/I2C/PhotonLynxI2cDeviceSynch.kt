package com.outoftheboxrobotics.photoncore.HAL.I2C

import com.qualcomm.hardware.lynx.commands.LynxCommand

interface PhotonLynxI2cDeviceSynch {
    fun setInterleavedCommand(command: LynxCommand<*>?)
    fun clearInterleavedCommand()
}
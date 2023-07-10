package com.outoftheboxrobotics.photoncore.HAL.I2C

import com.qualcomm.hardware.bosch.BHI260IMU
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple


class PhotonBHI260IMU(i2cDeviceSynchSimple: I2cDeviceSynchSimple?, deviceClientIsOwned: Boolean) : BHI260IMU(i2cDeviceSynchSimple, deviceClientIsOwned)
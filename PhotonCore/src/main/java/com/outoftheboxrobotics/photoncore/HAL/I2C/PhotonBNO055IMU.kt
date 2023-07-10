package com.outoftheboxrobotics.photoncore.HAL.I2C

import com.qualcomm.hardware.lynx.LynxEmbeddedIMU
import com.qualcomm.robotcore.hardware.I2cDeviceSynch

class PhotonBNO055IMU(deviceClient: I2cDeviceSynch?, deviceClientIsOwned: Boolean) :
    LynxEmbeddedIMU(deviceClient, deviceClientIsOwned)
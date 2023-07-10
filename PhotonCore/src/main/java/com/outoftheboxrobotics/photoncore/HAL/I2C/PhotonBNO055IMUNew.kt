package com.outoftheboxrobotics.photoncore.HAL.I2C

import com.qualcomm.hardware.lynx.LynxEmbeddedBNO055IMUNew
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple

class PhotonBNO055IMUNew(deviceClient: I2cDeviceSynchSimple?, deviceClientIsOwned: Boolean) :
    LynxEmbeddedBNO055IMUNew(deviceClient, deviceClientIsOwned)
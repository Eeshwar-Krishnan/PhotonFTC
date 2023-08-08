package com.outoftheboxrobotics.photoncore.HAL.I2C;

import com.qualcomm.hardware.lynx.LynxEmbeddedBNO055IMUNew;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;

public class PhotonBNO055IMUNew extends LynxEmbeddedBNO055IMUNew {

    public PhotonBNO055IMUNew(I2cDeviceSynchSimple deviceClient, boolean deviceClientIsOwned) {
        super(deviceClient, deviceClientIsOwned);
        // TODO: Actually replace the deviceClient
        // Idk what the intended mechanism is so I'm not intervening
    }
}

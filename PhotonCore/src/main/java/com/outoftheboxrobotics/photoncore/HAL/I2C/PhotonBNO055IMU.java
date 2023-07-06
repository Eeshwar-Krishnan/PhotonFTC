package com.outoftheboxrobotics.photoncore.HAL.I2C;

import com.qualcomm.hardware.lynx.LynxEmbeddedIMU;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

public class PhotonBNO055IMU extends LynxEmbeddedIMU {
    public PhotonBNO055IMU(I2cDeviceSynch deviceClient, boolean deviceClientIsOwned) {
        super(deviceClient, deviceClientIsOwned);
        // TODO: Actually replace the deviceClient
        // Idk what the intended mechanism is so I'm not intervening
    }
}

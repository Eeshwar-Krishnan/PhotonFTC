package com.outoftheboxrobotics.photoncore.HAL.I2C;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.hardware.bosch.BNO055IMUNew;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;

public class PhotonBHI260IMU  extends BHI260IMU {
    public PhotonBHI260IMU(I2cDeviceSynchSimple i2cDeviceSynchSimple, boolean deviceClientIsOwned) {
        super(i2cDeviceSynchSimple, deviceClientIsOwned);
        // TODO: Actually replace the deviceClient
        // Idk what the intended mechanism is so I'm not intervening
    }


}

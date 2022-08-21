package com.outoftheboxrobotics.photoncore.Neutrino.I2cCacheProfiles;

import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.bosch.BNO055IMU;

public class InternalIMUCacheProfile extends PhotonCore.I2CReadCache {
    public InternalIMUCacheProfile(BNO055IMU.Parameters parameters) {
        super(0, parameters.i2cAddr, 0X08, 44);
    }
}

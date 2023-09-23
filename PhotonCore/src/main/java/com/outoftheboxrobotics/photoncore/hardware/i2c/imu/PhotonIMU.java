package com.outoftheboxrobotics.photoncore.hardware.i2c.imu;


import com.outoftheboxrobotics.photoncore.hardware.i2c.PhotonInterleavableI2cDevice;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.concurrent.CompletableFuture;

public interface PhotonIMU extends IMU, PhotonInterleavableI2cDevice {

}

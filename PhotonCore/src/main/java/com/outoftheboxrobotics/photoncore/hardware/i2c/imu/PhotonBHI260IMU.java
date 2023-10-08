package com.outoftheboxrobotics.photoncore.hardware.i2c.imu;

import com.outoftheboxrobotics.photoncore.PhotonCommandBase;
import com.outoftheboxrobotics.photoncore.hardware.i2c.PhotonI2cDeviceSynch;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.lynx.commands.LynxMessage;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;
import com.qualcomm.robotcore.hardware.QuaternionBasedImuHelper;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;

import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.internal.hardware.android.AndroidBoard;
import org.firstinspires.ftc.robotcore.internal.hardware.android.GpioPin;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.concurrent.CompletableFuture;

public class PhotonBHI260IMU extends BHI260IMU implements PhotonIMU{
    private static final String TAG = "PhotonBHI260IMU";
    private final QuaternionBasedImuHelper helper = new QuaternionBasedImuHelper(parameters.imuOrientationOnRobot);
    private static final double QUATERNION_SCALE_FACTOR = Math.pow(2, -14);

    public PhotonBHI260IMU(PhotonI2cDeviceSynch deviceClient, boolean deviceClientIsOwned) {
        super((I2cDeviceSynchSimple) deviceClient, deviceClientIsOwned);
    }
    @Override
    public String getDeviceName() {
        return TAG;
    }

    @Override
    public Manufacturer getManufacturer() {
        return null;
    }

    @Override
    public void scheduleInterleavedCommand(PhotonCommandBase<? extends LynxMessage> command) {
        ((PhotonI2cDeviceSynch)getDeviceClient()).scheduleInterleavedCommand(command);

    }

    @Override
    public LynxMessage getResult() {
        return ((PhotonI2cDeviceSynch)getDeviceClient()).getResult();
    }
}

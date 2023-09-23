package com.outoftheboxrobotics.photoncore.hardware.i2c.imu;

import com.outoftheboxrobotics.photoncore.PhotonCommandBase;
import com.outoftheboxrobotics.photoncore.hardware.i2c.PhotonI2cDeviceSynch;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMUNew;
import com.qualcomm.hardware.lynx.commands.LynxMessage;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;
import com.qualcomm.robotcore.hardware.QuaternionBasedImuHelper;
import com.qualcomm.robotcore.hardware.TimestampedData;

import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.concurrent.CompletableFuture;

public class PhotonBNO055IMUNew extends BNO055IMUNew implements PhotonIMU{
    private static final String TAG = "PhotonBNO055IMUNew";
    private final QuaternionBasedImuHelper helper = new QuaternionBasedImuHelper(parameters.imuOrientationOnRobot);

    public PhotonBNO055IMUNew(PhotonI2cDeviceSynch deviceClient, boolean deviceClientIsOwned) {
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
    protected static class VectorData
    {
        public TimestampedData data;
        public    float             scale;
        protected ByteBuffer buffer;

        public VectorData(TimestampedData data, float scale)
        {
            this.data = data;
            this.scale = scale;
            buffer = ByteBuffer.wrap(data.data).order(ByteOrder.LITTLE_ENDIAN);
        }

        public float next()
        {
            return buffer.getShort() / scale;
        }
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

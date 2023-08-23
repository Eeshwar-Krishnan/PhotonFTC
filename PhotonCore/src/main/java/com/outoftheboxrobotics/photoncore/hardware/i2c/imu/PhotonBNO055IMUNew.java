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
    private final PhotonI2cDeviceSynch deviceClient;
    private final QuaternionBasedImuHelper helper = new QuaternionBasedImuHelper(parameters.imuOrientationOnRobot);

    public PhotonBNO055IMUNew(PhotonI2cDeviceSynch deviceClient, boolean deviceClientIsOwned) {
        super((I2cDeviceSynchSimple) deviceClient, deviceClientIsOwned);
        this.deviceClient=deviceClient;
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
        deviceClient.scheduleInterleavedCommand(command);
    }

    @Override
    public CompletableFuture<? extends LynxMessage> getResponse() {
        return deviceClient.getResponse();
    }

    @Override
    public CompletableFuture<YawPitchRollAngles> getRobotYawPitchRollAnglesAsync() {

        return deviceClient.readTimeStampedAsync(BNO055IMU.Register.QUA_DATA_W_LSB.bVal, 8).thenApply(timestampedData -> {
            boolean receivedAllZeros = true;
            for (byte b: timestampedData.data) {
                if (b != 0) {
                    receivedAllZeros = false;
                    break;
                }
            }

            if (receivedAllZeros) {
                // All zeros is not a valid quaternion.
                return null;
            }
            VectorData vector = new VectorData(timestampedData, (1 << 14));
            return new Quaternion(vector.next(), vector.next(), vector.next(), vector.next(), vector.data.nanoTime);
        }).thenApply(quaternion ->
        {
            if(quaternion==null) return null;
            return helper.getRobotYawPitchRollAngles(TAG,(() -> quaternion));
        });
    }
}

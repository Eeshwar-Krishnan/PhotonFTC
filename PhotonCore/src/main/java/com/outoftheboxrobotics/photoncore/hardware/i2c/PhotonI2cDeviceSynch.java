package com.outoftheboxrobotics.photoncore.hardware.i2c;

import com.qualcomm.robotcore.hardware.TimestampedData;

import java.util.concurrent.CompletableFuture;

public interface PhotonI2cDeviceSynch extends PhotonInterleavableI2cDevice{


    CompletableFuture<TimestampedData> readTimeStampedAsync(int ireg, int creg);

    class TimestampedCompletableFuture<T>
    {
        public TimestampedData timestampedData;
        public CompletableFuture<T> future;
        public TimestampedCompletableFuture(TimestampedData timestampedData, CompletableFuture<T> future)
        {
            this.timestampedData=timestampedData;
            this.future=future;
        }
    }
}

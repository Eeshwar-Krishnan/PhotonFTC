package org.outoftheboxrobotics.neutrinoi2c.MB1242;

import android.content.Context;

import com.qualcomm.ftccommon.FtcEventLoop;
import com.qualcomm.hardware.lynx.LynxI2cDeviceSynch;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.hardware.lynx.commands.core.LynxI2cReadMultipleBytesCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxI2cReadStatusQueryCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxI2cReadStatusQueryResponse;
import com.qualcomm.robotcore.eventloop.EventLoop;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImplOnSimple;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.TypeConversion;

import org.firstinspires.ftc.ftccommon.external.OnCreateEventLoop;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.outoftheboxrobotics.neutrinoi2c.Reflection.ReflectionUtils;

import java.lang.reflect.Field;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicLong;

@I2cDeviceType
@DeviceProperties(
        name = "AsyncMB1242",
        description = "Ultrasonic distance sensor",
        xmlTag = "AsyncMB1242"
)
public class AsyncMB1242 extends I2cDeviceSynchDevice<I2cDeviceSynch> implements DistanceSensor {
    private long lastRun = 0;

    private long runDelayMs = 40;

    private short lastReading = 0;


    public AsyncMB1242(I2cDeviceSynch i2cDeviceSynch) {
        super(i2cDeviceSynch, true);

        deviceClient.setI2cAddress(I2cAddr.create7bit(0x70));
        registerArmingStateCallback(false);
        deviceClient.engage();
    }

    @Override
    public double getDistance(DistanceUnit unit) {
        if(System.currentTimeMillis() > (lastRun + runDelayMs)){
            lastRun = System.currentTimeMillis();
            deviceClient.write(TypeConversion.intToByteArray(0x51));
        }
        if(System.currentTimeMillis() > (lastRun + 20)) {
            lastReading = TypeConversion.byteArrayToShort(deviceClient.read( 2));
        }
        return unit.fromCm(lastReading);
    }

    public void setRunDelayMs(long runDelayMs) {
        this.runDelayMs = runDelayMs;
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }

    @Override
    public String getDeviceName() {
        return "MB1242 sensor";
    }

    @Override
    public String getConnectionInfo() {
        return "Connected lol";
    }

    @Override
    public int getVersion() {
        return 1;
    }

    @Override
    protected boolean doInitialize() {
        return true;
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {

    }

    @Override
    public void close() {

    }
}

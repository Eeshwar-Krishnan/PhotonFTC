package com.outoftheboxrobotics.photoncore.Neutrino.MB1242;

import android.content.Context;

import com.outoftheboxrobotics.photoncore.PhotonLynxModule;
import com.outoftheboxrobotics.photoncore.ReflectionUtils;
import com.qualcomm.ftccommon.FtcEventLoop;
import com.qualcomm.hardware.lynx.LynxI2cDeviceSynch;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.hardware.lynx.LynxUsbDevice;
import com.qualcomm.hardware.lynx.LynxUsbDeviceDelegate;
import com.qualcomm.hardware.lynx.LynxUsbDeviceImpl;
import com.qualcomm.hardware.lynx.Supplier;
import com.qualcomm.hardware.lynx.commands.LynxCommand;
import com.qualcomm.hardware.lynx.commands.LynxRespondable;
import com.qualcomm.hardware.lynx.commands.core.LynxI2cReadMultipleBytesCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxI2cReadStatusQueryCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxI2cReadStatusQueryResponse;
import com.qualcomm.hardware.lynx.commands.core.LynxI2cWriteReadMultipleBytesCommand;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerImpl;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImplOnSimple;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;
import com.qualcomm.robotcore.hardware.TimestampedData;
import com.qualcomm.robotcore.hardware.TimestampedI2cData;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.hardware.usb.RobotUsbDevice;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.TypeConversion;

import org.firstinspires.ftc.ftccommon.external.OnCreateEventLoop;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.atomic.AtomicBoolean;

@I2cDeviceType
@DeviceProperties(
        name = "Photon MB1242",
        description = "ultrasonic distance sensor",
        xmlTag = "MB1242"
)
public class MB1242Ex extends I2cDeviceSynchDevice<I2cDeviceSynch> implements DistanceSensor {
    private static OpModeManagerImpl opModeManager;
    private LynxModule module;
    private int bus;
    private I2cAddr address;
    private long lastRun = 0;

    private short lastReading = 0;

    public MB1242Ex(I2cDeviceSynch i2cDeviceSynch) {
        super(i2cDeviceSynch, true);

        deviceClient.setI2cAddress(I2cAddr.create7bit(0x70));
        registerArmingStateCallback(false);
        deviceClient.engage();

        setupVariables(i2cDeviceSynch);
    }

    public double getDistanceAsync(DistanceUnit unit) {
        if(System.currentTimeMillis() > (lastRun + 20)) {
            try {
                module.acquireI2cLockWhile(() -> {
                    LynxCommand<?> tx = new LynxI2cWriteReadMultipleBytesCommand(module, bus, address, 0, 2);

                    tx.send();

                    byte[] data = pollForReadResult(2);

                    if(data.length == 2){
                        lastReading = TypeConversion.byteArrayToShort(data);
                        deviceClient.write(TypeConversion.intToByteArray(0x51));
                        lastRun = System.currentTimeMillis();
                    }
                    return null;
                });
            } catch (InterruptedException e) {
                e.printStackTrace();
            } catch (RobotCoreException e) {
                e.printStackTrace();
            } catch (LynxNackException e) {
                e.printStackTrace();
            }
        }
        return unit.fromCm(lastReading);
    }

    @Override
    public double getDistance(DistanceUnit unit) {
        try {
            module.acquireI2cLockWhile(() -> {
                LynxCommand<?> tx = new LynxI2cReadMultipleBytesCommand(module, bus, address, 2);
                tx.send();

                byte[] data = pollForReadResult(2);

                while(data.length == 0){
                    tx = new LynxI2cReadMultipleBytesCommand(module, bus, address, 2);
                    try {
                        tx.send();
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    } catch (LynxNackException e) {
                        e.printStackTrace();
                    }
                    data = pollForReadResult(2);
                }

                if(data.length == 2){
                    lastReading = TypeConversion.byteArrayToShort(data);
                    deviceClient.write(TypeConversion.intToByteArray(0x51));
                    lastRun = System.currentTimeMillis();
                }
                return null;
            });
        } catch (InterruptedException e) {
            e.printStackTrace();
        } catch (RobotCoreException e) {
            e.printStackTrace();
        } catch (LynxNackException e) {
            e.printStackTrace();
        }

        return unit.fromCm(lastReading);
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

    private void setupVariables(I2cDeviceSynch i2cDeviceSynch){
        LynxI2cDeviceSynch device = null;
        try {
            I2cDeviceSynchImplOnSimple simple = (I2cDeviceSynchImplOnSimple) i2cDeviceSynch;
            //Cool first part done
            //Now we can safely make the assumption that the underlying i2cdevicesynchsimple is a lynxi2cdevicesynch
            Field field = ReflectionUtils.getField(simple.getClass(), "i2cDeviceSynchSimple");
            field.setAccessible(true);
            device = (LynxI2cDeviceSynch) field.get(simple);
            //Lets also bump up the bus speed while we are here
            //Tbh it doesn't affect anything when just reading 2 bytes but why not
            device.setBusSpeed(LynxI2cDeviceSynch.BusSpeed.FAST_400K);
        } catch (IllegalAccessException e) {
            e.printStackTrace();
        }
        LynxModule module = null;
        try {
            //Module this is being run on, sometimes the module doesn't exist
            module = (LynxModule) ReflectionUtils.getField(device.getClass(), "module").get(device);
        } catch (IllegalAccessException e) {
            e.printStackTrace();
        }

        this.module = module;

        int bus = 0;
        try {
            //Bus that the i2c device is on. Why is this one the only one IntelliJ is mad at?
            bus = (int) ReflectionUtils.getField(device.getClass(), "bus").get(device);
        } catch (IllegalAccessException e) {
            e.printStackTrace();
        }

        this.bus = bus;

        this.address = I2cAddr.create7bit(0x70);
    }

    protected byte[] pollForReadResult(int creg) {
        // Poll until the data is available
        boolean keepTrying = true;
        while (keepTrying)
        {
            LynxI2cReadStatusQueryCommand readStatus = new LynxI2cReadStatusQueryCommand(module, this.bus, creg);
            try {
                LynxI2cReadStatusQueryResponse response = readStatus.sendReceive();
                long now = System.nanoTime();
                return response.getBytes();
            }
            catch (LynxNackException e)
            {
                switch (e.getNack().getNackReasonCodeAsEnum())
                {
                    case I2C_MASTER_BUSY:               // TODO: REVIEW: is this ever actually returned in this situation?
                    case I2C_OPERATION_IN_PROGRESS:
                        // We used to sleep for 3ms while waiting for the result to avoid a "busy loop", but that
                        // caused a serious performance hit over what we could get otherwise, at least on the CH.
                        // Besides, we're not *truly* busy looping, we still end up waiting for the module's response
                        // and what not.

                        //try { Thread.sleep(msBusyWait); } catch (InterruptedException ignored) { Thread.currentThread().interrupt(); }
                        continue;// This is an internal error of some sort
                    default:
                        keepTrying = false;
                        break;
                }
            }
            catch (InterruptedException|RuntimeException e)
            {
                keepTrying = false;
            }
        }
        return new byte[0];
    }
}
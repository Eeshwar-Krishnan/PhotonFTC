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
public class AsyncMB1242 extends I2cDeviceSynchDevice<I2cDeviceSynch> implements DistanceSensor, OpModeManagerNotifier.Notifications, Runnable {
    private final AtomicLong lastRun = new AtomicLong(0);

    private static EventLoop eventLoopStatic;

    private AtomicLong minRunDelayMs = new AtomicLong(20);

    private final AtomicBoolean running = new AtomicBoolean(true), enabled = new AtomicBoolean(false);
    private final AtomicInteger rangeMM = new AtomicInteger(0);

    private final LynxModule module;
    private final int bus;
    private final I2cAddr address;
    public AsyncMB1242(I2cDeviceSynch i2cDeviceSynch) {
        super(i2cDeviceSynch, true);

        deviceClient.setI2cAddress(I2cAddr.create7bit(0x70));
        registerArmingStateCallback(false);
        deviceClient.engage();

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
            //Module this is being run on, sometimes the module doesn't exist??? but it seems to coincide with ESD so
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

        I2cAddr i2cAddr = null;
        try {
            //I2C address of the sensor. Technically this can change but I'm assuming by the time this constructor is called its done changing
            i2cAddr = (I2cAddr) ReflectionUtils.getField(device.getClass(), "i2cAddr").get(device);
        } catch (IllegalAccessException e) {
            e.printStackTrace();
        }

        this.address = i2cAddr;
    }

    @OnCreateEventLoop
    public static void attachEventLoop(Context context, FtcEventLoop eventLoop) {
        eventLoopStatic = eventLoop;
    }

    @Override
    public double getDistance(DistanceUnit unit) {
        return unit.fromMm(rangeMM.get());
    }

    public void setMinRunDelayMs(long minRunDelayMs) {
        this.minRunDelayMs.set(minRunDelayMs);
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
        return "";
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

    @Override
    public void onOpModePreInit(OpMode opMode) {
        if(eventLoopStatic.getOpModeManager().getActiveOpModeName().equals("$Stop$Robot$")){
            return;
        }
        running.set(true);
        new Thread(this).start();
    }

    @Override
    public void onOpModePreStart(OpMode opMode) {

    }

    @Override
    public void onOpModePostStop(OpMode opMode) {
        if(eventLoopStatic.getOpModeManager().getActiveOpModeName().equals("$Stop$Robot$")){
            return;
        }
        running.set(false);
    }

    public long getLastRun() {
        return lastRun.get();
    }

    @Override
    public void run() {
        while(running.get()){
            if(!enabled.get())
                continue;

            final long[] timeTaken = {0};

            deviceClient.write(TypeConversion.intToByteArray(0x51)); //Send ping signal

            boolean readSuccess = false;
            AtomicBoolean sentReadRequest = new AtomicBoolean(false);

            while(!readSuccess) {
                final boolean[] waitingForNewData = {false};
                try {
                    readSuccess = module.acquireI2cLockWhile(() -> {
                        if(!sentReadRequest.get()) {
                            LynxI2cReadMultipleBytesCommand command = new LynxI2cReadMultipleBytesCommand(module, bus, address, 2);
                            command.send();
                            sentReadRequest.set(true);
                        }

                        LynxI2cReadStatusQueryCommand command2 = new LynxI2cReadStatusQueryCommand(module, bus, 2);
                        LynxI2cReadStatusQueryResponse response = command2.sendReceive();

                        if(response.getBytes().length == 0){
                            waitingForNewData[0] = true;
                            return false;
                        }else{
                            rangeMM.set((int) DistanceUnit.MM.fromCm(TypeConversion.byteArrayToShort(response.getBytes())));
                            long now = System.currentTimeMillis();
                            timeTaken[0] = now - lastRun.get();
                            lastRun.set(System.currentTimeMillis());
                            return true;
                        }
                    });
                } catch (InterruptedException | RobotCoreException e) {
                    e.printStackTrace();
                } catch (LynxNackException e) {
                    try {
                        Thread.sleep(5);
                    } catch (InterruptedException interruptedException) {
                        interruptedException.printStackTrace();
                    }
                }
                if(waitingForNewData[0]){
                    try {
                        Thread.sleep(10);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }
            }

            if(timeTaken[0] < minRunDelayMs.get()){
                try {
                    //Make sure we aren't running faster then 20 ms per loop
                    //Running faster could theoretically cause issues
                    Thread.sleep(minRunDelayMs.get() - timeTaken[0]);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        }
    }

    public void enable(){
        enabled.set(true);
    }

    public void disable(){
        enabled.set(false);
    }
}

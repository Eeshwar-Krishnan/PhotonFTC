package org.outoftheboxrobotics.neutrinoi2c.Rev2mDistanceSensor;

import android.content.Context;

import com.qualcomm.ftccommon.FtcEventLoop;
import com.qualcomm.hardware.lynx.LynxI2cDeviceSynch;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.hardware.lynx.commands.LynxCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxI2cReadStatusQueryCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxI2cReadStatusQueryResponse;
import com.qualcomm.hardware.lynx.commands.core.LynxI2cWriteReadMultipleBytesCommand;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.EventLoop;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImplOnSimple;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.TypeConversion;

import org.firstinspires.ftc.ftccommon.external.OnCreateEventLoop;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.outoftheboxrobotics.neutrinoi2c.Reflection.ReflectionUtils;

import java.lang.reflect.Field;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicLong;

public class AsyncRev2MSensor implements OpModeManagerNotifier.Notifications, Runnable{
    private static final int RESULT_INTERRUPT_STATUS = 0x13, RESULT_RANGE_STATUS = 0x14, SYSTEM_INTERRUPT_CLEAR = 0x0B;

    public enum AccuracyMode{
        MODE_HIGH_SPEED,
        MODE_BALANCED,
        MODE_HIGH_ACCURACY
    }

    private static EventLoop staticEventLoop;

    private final LynxModule module;
    private final int bus;
    private final I2cAddr address;
    private final Rev2mDistanceSensor sensor;

    private long minimumTimingBudget = 33;
    private final AtomicLong timingBudget;

    private final AtomicLong prevRun = new AtomicLong(0);
    private final AtomicInteger range = new AtomicInteger(0);

    private final AtomicBoolean running = new AtomicBoolean(true);
    private final AtomicBoolean enabled = new AtomicBoolean(true);

    private Method setTimingBudgetMethod;

    private AtomicBoolean hasNewData = new AtomicBoolean(false);

    @OnCreateEventLoop
    public static void attachEventLoop(Context context, FtcEventLoop eventLoop) {
        staticEventLoop = eventLoop;
        //Since this is threaded, we should make checks to see when the opmode stops and make sure the thread is stopped on opmode stop
        //Runaway threads, especially those manually sending i2c requests like we are, is a horrible idea
    }

    public AsyncRev2MSensor(Rev2mDistanceSensor sensor){
        LynxI2cDeviceSynch device = null;
        try {
            /**
             * Getting data here sucks a lil
             *
             * Internally, probably due to back compatibility? There are a ton of interfaces and classes that we have to get through
             *
             * Basically, there is a fake I2CDeviceSynch class that takes in a I2CDeviceSynchSimple class and adds heartbeat and readwindow except it totally doesn't -_-
             * Then that is called by the hardware factory which zzzzzzzzzz
             * yeah i fell asleep too anyway i figured it out finally i think
             * (text me or something for the more detailed explanation/rant of what goes on here)
             */

            //Most likely this is being used on a control hub, so we are going to assume the passed i2cdevicesynch is a I2cDeviceSynchImplOnSimple
            //Probably
            //Hopefully
            Field field = ReflectionUtils.getField(sensor.getClass(), "deviceClient");
            field.setAccessible(true);
            I2cDeviceSynchImplOnSimple simple = (I2cDeviceSynchImplOnSimple) field.get(sensor);

            //Cool first part done
            //Now we can safely make the assumption that the underlying i2cdevicesynchsimple is a lynxi2cdevicesynch
            field = ReflectionUtils.getField(simple.getClass(), "i2cDeviceSynchSimple");
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
            //Fairly safe because nothing in the 2m driver calls for it to change address randomly
            i2cAddr = (I2cAddr) ReflectionUtils.getField(device.getClass(), "i2cAddr").get(device);
        } catch (IllegalAccessException e) {
            e.printStackTrace();
        }

        this.address = i2cAddr;

        try {
            //I was going to implement the timing budget calculations but FIRST did it for me. Thanks! I'll just steal access to that method
            StringBuilder stringBuilder = new StringBuilder();
            for(Method m : sensor.getClass().getSuperclass().getDeclaredMethods()){
                stringBuilder.append(m.getName()).append(",");
            }
            RobotLog.ii("Test", stringBuilder.toString());
            Method m = sensor.getClass().getSuperclass().getDeclaredMethod("getMeasurementTimingBudget");
            m.setAccessible(true);
            minimumTimingBudget = (((long) m.invoke(sensor))+2)/1000; //Fudge factor to account for small timing offsets
        } catch (NoSuchMethodException | IllegalAccessException | InvocationTargetException e) {
            e.printStackTrace();
        }

        try {
            //Also was going to reimplement, but strangely this is coded but not available to the user
            setTimingBudgetMethod = sensor.getClass().getSuperclass().getDeclaredMethod("setMeasurementTimingBudget", long.class);
            setTimingBudgetMethod.setAccessible(true);
        } catch (NoSuchMethodException e) {
            e.printStackTrace();
        }

        staticEventLoop.getOpModeManager().registerListener(this);

        timingBudget = new AtomicLong(minimumTimingBudget);

        this.sensor = sensor;

        new Thread(this).start();
    }

    public double getDistance(DistanceUnit unit){
        hasNewData.set(false);
        double range = (double)this.range.get();
        if (unit == DistanceUnit.CM) {
            return range / 10;
        } else if (unit == DistanceUnit.METER) {
            return range / 1000;
        } else if (unit == DistanceUnit.INCH) {
            return range / 25.4;
        } else {
            return range;
        }
    }

    @Override
    public void onOpModePreInit(OpMode opMode) {
        running.set(true);
    }

    @Override
    public void onOpModePreStart(OpMode opMode) {

    }

    @Override
    public void onOpModePostStop(OpMode opMode) {
        running.set(false);
        staticEventLoop.getOpModeManager().unregisterListener(this); //This object will most likely be destroyed after stop, so lets just deregister it
    }

    @Override
    public void run() {
        while(running.get()){
            if(!enabled.get()){
                continue;
            }
            //RobotLog.ii("2M Status", "Awaiting Read");

            while(System.currentTimeMillis() - prevRun.get() < timingBudget.get()) {
                //No point in running now, most likely. Sensor is either processing the new data or waiting for a new measurement to start
            }
            //RobotLog.ii("2M Status", "Started Read");

            //We can make an assumption at this point that there is new data available
            //Since its been longer then the timing budget, so the sensor would have taken a measurement
            //So lets just get it and ignore the check to see if the data is new
            boolean successful = false;
            try {
                successful = module.acquireI2cLockWhile(() -> {//Acquire the i2c lock to write data
                    LynxCommand<?> tx = new LynxI2cWriteReadMultipleBytesCommand(module, bus, address, RESULT_RANGE_STATUS + 10, 2);
                    tx.send(); //Request to read 2 bytes starting at register 30
                    //These two bytes are the range the sensor has last read
                    return true;
                });
            } catch (InterruptedException | RobotCoreException | LynxNackException e) {
                e.printStackTrace();
            }

            if(!successful){
                continue;
                //error from the hub, happens sometimes, just try again later
            }

            //RobotLog.ii("2M Status", "Acquired Lock");

            boolean hasResponse = false;
            while(!hasResponse) {
                try {
                    hasResponse = module.acquireI2cLockWhile(() -> {
                        LynxI2cReadStatusQueryCommand command = new LynxI2cReadStatusQueryCommand(module, bus, 2);
                        LynxI2cReadStatusQueryResponse response = command.sendReceive(); //Read the 2 bytes we asked for previously
                        range.set(TypeConversion.byteArrayToShort(response.getBytes()));
                        hasNewData.set(true);
                        prevRun.set(System.currentTimeMillis()); //Timer starts the moment we read the data, we can guarantee that after
                        //the timing budget window there is fresh data
                        //After all, we are receiving the data after a ~5ms delay so the sensor should have plenty of time to read new data
                        //RobotLog.ii("2M Status", "Read Data!");
                        return true;
                    });
                } catch (InterruptedException | RobotCoreException e) {
                    e.printStackTrace();
                } catch (LynxNackException e) {
                    switch (e.getNack().getNackReasonCodeAsEnum())
                    {
                        case I2C_MASTER_BUSY: //This doesn't ever seem to get thrown by the lynx but
                            //the sdk has it so I might as well have it?
                            //I assume this is legacy from the MR controllers, but it could also be called by the hub in rare circumstances
                        case I2C_OPERATION_IN_PROGRESS:
                            //Data is still in flight, lets wait 5 ms then try again
                            //The wait will make this slightly slower compared to SDK but is worth it
                            //To allow user commands waiting for i2c/command lock to get it

                            //TODO: There has got to be a more efficient way to determine that data is available
                            //Possibly we could time the average response latency of the 2m and wait that amount
                            //Although that would be difference from hub to hub and from chub to ehub etc
                            //RobotLog.ii("2M Status", "NACK");
                            try { Thread.sleep(5); } catch (InterruptedException ignored) { hasResponse = true;}
                            continue;
                        case I2C_NO_RESULTS_PENDING:
                            //We definitely asked for data, so if it doesn't have anything something happened to our request
                            //while in flight. We should just try again
                        default:
                            //Some error occurred, just break out of here and try again
                            hasResponse = true;
                            break;
                    }
                }
            }
        }
    }

    /**
     * Stops the sensor from running
     */
    public void disable(){
        this.enabled.set(false);
    }

    /**
     * Turns the sensor back on
     */
    public void enable(){
        this.enabled.set(true);
    }

    /**
     * Sets accuracy mode of the 2m sensor
     * @param mode
     * @return
     */
    public boolean setSensorAccuracyMode(AccuracyMode mode){
        if(mode == AccuracyMode.MODE_HIGH_SPEED){
            return setMeasurementIntervalMs(33);
        }
        if(mode == AccuracyMode.MODE_BALANCED){
            return setMeasurementIntervalMs(75);
        }
        if(mode == AccuracyMode.MODE_HIGH_ACCURACY){
            return setMeasurementIntervalMs(150);
        }
        return false;
    }

    /**
     * How many ms the sensor should wait between reads
     *
     * This method doesn't just adjust run rate, the longer it can wait between reads the more accurate the data will be
     * This is because the driver will instruct the 2m sensor to spend longer processing the data before returning it
     * Generally an N times increase in timing budget means the standard deviation of measurements will decrease by the square root of N
     *
     * The minimum time between reads is 20 ms, however it is implemented here as 33 ms which seems to be the lowest stable measurement budget. More testing is needed here
     * The maximum time between reads allowed is 200 ms, as above this amount the sensor will not gain any more benefit from processing, at least as far as I can tell from the limited driver information
     *
     * @param measurementIntervalMs The amount of time, in milliseconds, that should be allocated between reads
     * @return true if the timing budget was successfully set
     */
    public boolean setMeasurementIntervalMs(long measurementIntervalMs){
        if(measurementIntervalMs < minimumTimingBudget){
            return false;
        }
        if(measurementIntervalMs > 200){
            return false;
        }
        this.timingBudget.set(measurementIntervalMs+2);//Fudge factor here, at worst the sensor seems to be +- 2ms late but that has only ever happened once. Could reduce
        internalSetMeasurementBudget(measurementIntervalMs);
        return true;
    }

    private void internalSetMeasurementBudget(long measurementBudget){
        try {
            //Method takes microseconds but nobody uses microseconds so lets convert it from milliseconds
            setTimingBudgetMethod.invoke(sensor, measurementBudget * 1000);
        } catch (IllegalAccessException | InvocationTargetException e) {
            e.printStackTrace();
        }
    }

    /**
     * Returns true if new data has been acquired since the last call to getDistance()
     */
    public boolean hasNewData(){
        return hasNewData.get();
    }

    /**
     * Gets the timestamp of the last measurement received
     * @return the timestamp, accurate to within 10ms
     */
    public long getLastMeasurementTimestamp(){
        return prevRun.get();
    }
}

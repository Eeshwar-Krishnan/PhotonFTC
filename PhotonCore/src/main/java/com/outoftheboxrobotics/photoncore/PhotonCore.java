package com.outoftheboxrobotics.photoncore;

import android.content.Context;


import com.outoftheboxrobotics.photoncore.Neutrino.Rev2MSensor.Rev2mDistanceSensorEx;
import com.outoftheboxrobotics.photoncore.Neutrino.RevColorSensor.RevColorSensorV3Ex;
import com.qualcomm.ftccommon.FtcEventLoop;
import com.qualcomm.hardware.lynx.LynxI2cDeviceSynch;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxUnsupportedCommandException;
import com.qualcomm.hardware.lynx.LynxUsbDevice;
import com.qualcomm.hardware.lynx.LynxUsbDeviceDelegate;
import com.qualcomm.hardware.lynx.LynxUsbDeviceImpl;
import com.qualcomm.hardware.lynx.commands.LynxCommand;
import com.qualcomm.hardware.lynx.commands.LynxDatagram;
import com.qualcomm.hardware.lynx.commands.LynxMessage;
import com.qualcomm.hardware.lynx.commands.LynxRespondable;
import com.qualcomm.hardware.lynx.commands.core.LynxI2cReadStatusQueryCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxI2cWriteReadMultipleBytesCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxSetMotorConstantPowerCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxSetServoPulseWidthCommand;
import com.qualcomm.hardware.lynx.commands.standard.LynxAck;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;
import com.qualcomm.robotcore.hardware.usb.RobotUsbDevice;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.ftccommon.external.OnCreateEventLoop;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeManagerImpl;
import org.firstinspires.ftc.robotcore.internal.usb.exception.RobotUsbException;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;

public class PhotonCore implements Runnable, OpModeManagerNotifier.Notifications {
    protected static final PhotonCore instance = new PhotonCore();
    protected AtomicBoolean enabled, threadEnabled;

    private List<LynxModule> modules;
    private Thread thisThread = null;
    private Object syncLock;

    private final Object messageSync = new Object();

    private RobotUsbDevice robotUsbDevice;

    public static LynxModule CONTROL_HUB, EXPANSION_HUB;

    private OpModeManagerImpl opModeManager;

    public static class ExperimentalParameters{
        private final AtomicBoolean singlethreadedOptimized = new AtomicBoolean(true);
        private final AtomicInteger maximumParallelCommands = new AtomicInteger(8);

        public void setSinglethreadedOptimized(boolean state){
            this.singlethreadedOptimized.set(state);
        }

        public boolean setMaximumParallelCommands(int maximumParallelCommands){
            if(maximumParallelCommands > 9 || maximumParallelCommands <= 0){
                return false;
            }
            this.maximumParallelCommands.set(maximumParallelCommands);
            return true;
        }
    }

    public static ExperimentalParameters experimental = new ExperimentalParameters();

    public PhotonCore(){
        CONTROL_HUB = null;
        EXPANSION_HUB = null;
        enabled = new AtomicBoolean(false);
        threadEnabled = new AtomicBoolean(false);
    }

    public static void enable(){
        instance.enabled.set(true);
        if(CONTROL_HUB.getBulkCachingMode() == LynxModule.BulkCachingMode.OFF){
            CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    public static void disable(){
        instance.enabled.set(false);
    }

    @OnCreateEventLoop
    public static void attachEventLoop(Context context, FtcEventLoop eventLoop) {
        eventLoop.getOpModeManager().registerListener(instance);
        instance.opModeManager = eventLoop.getOpModeManager();
    }

    protected static boolean registerSend(LynxCommand command) throws LynxUnsupportedCommandException, InterruptedException {

        PhotonLynxModule photonModule = (PhotonLynxModule) command.getModule();

        synchronized (instance.messageSync) {
            while (((((PhotonLynxModule)CONTROL_HUB).getUnfinishedCommands().size() + (EXPANSION_HUB == null ? 0 : ((PhotonLynxModule)EXPANSION_HUB).getUnfinishedCommands().size())) > experimental.maximumParallelCommands.get()));

            if(!experimental.singlethreadedOptimized.get()) {
                boolean noSimilar = false;
                while (!noSimilar) {
                    noSimilar = true;
                    for (LynxRespondable respondable : photonModule.getUnfinishedCommands().values()) {
                        if (instance.isSimilar(respondable, command)) {
                            noSimilar = false;
                        }
                    }
                }
            }

            byte messageNum = photonModule.getNewMessageNumber();

            command.setMessageNumber(messageNum);

            try {
                LynxDatagram datagram = new LynxDatagram(command);
                command.setSerialization(datagram);

                if (command.isAckable() || command.isResponseExpected()) {
                    photonModule.getUnfinishedCommands().put(command.getMessageNumber(), (LynxRespondable) command);
                }

                byte[] bytes = datagram.toByteArray();

                double msLatency = 0;
                synchronized (instance.syncLock) {
                    long start = System.nanoTime();
                    instance.robotUsbDevice.write(bytes);
                    long stop = System.nanoTime();
                    msLatency = (stop - start) * 1.0e-6;
                }
                //RobotLog.ii("PhotonCore", "Wrote " + bytes.length + " bytes " + photonModule.getUnfinishedCommands().size() + " | " + (msLatency));

                if (shouldAckImmediately(command)) {
                    command.onAckReceived(new LynxAck(photonModule, false));
                }
            } catch (LynxUnsupportedCommandException | RobotUsbException e) {
                e.printStackTrace();
            }
        }

        return true;
    }

    protected static boolean shouldParallelize(LynxCommand command){
        return (command instanceof LynxSetMotorConstantPowerCommand) ||
                (command instanceof LynxSetServoPulseWidthCommand);
    }

    protected static boolean shouldAckImmediately(LynxCommand command){
        return (command instanceof LynxSetMotorConstantPowerCommand) ||
                (command instanceof LynxSetServoPulseWidthCommand);
    }

    private boolean isSimilar(LynxRespondable respondable1, LynxRespondable respondable2){
        return (respondable1.getDestModuleAddress() == respondable2.getDestModuleAddress()) &&
                (respondable1.getCommandNumber() == respondable2.getCommandNumber());
    }

    protected static LynxMessage getCacheResponse(LynxCommand command){
        return null;
    }

    @Override
    public void run() {
        while(threadEnabled.get()){

            try {
                Thread.sleep(5);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

    @Override
    public void onOpModePreInit(OpMode opMode) {
        if(opModeManager.getActiveOpModeName().equals(OpModeManager.DEFAULT_OP_MODE_NAME)){
            return;
        }

        HardwareMap map = opMode.hardwareMap;

        boolean replacedPrev = false;
        for(LynxModule module : map.getAll(LynxModule.class)){
            if(module instanceof PhotonLynxModule){
                replacedPrev = true;
            }
        }
        if(replacedPrev){
            HashMap<String, HardwareDevice> toRemove = new HashMap<>();
            for(LynxModule module : map.getAll(LynxModule.class)){
                if(!(module instanceof PhotonLynxModule)){
                    toRemove.put((String) map.getNamesOf(module).toArray()[0], module);
                }
            }
            for(String s : toRemove.keySet()){
                map.remove(s, toRemove.get(s));
            }
        }

        instance.modules = map.getAll(LynxModule.class);
        ArrayList<String> moduleNames = new ArrayList<>();
        HashMap<LynxModule, PhotonLynxModule> replacements = new HashMap<>();
        for(LynxModule module : instance.modules){
            moduleNames.add((String) map.getNamesOf(module).toArray()[0]);
        }

        for(String s : moduleNames){
            LynxModule module = (LynxModule) map.get(LynxModule.class, s);
            if(module instanceof PhotonLynxModule){
                continue;
            }
            try {
                PhotonLynxModule photonLynxModule = new PhotonLynxModule(
                        (LynxUsbDevice) ReflectionUtils.getField(module.getClass(), "lynxUsbDevice").get(module),
                        (Integer)ReflectionUtils.getField(module.getClass(), "moduleAddress").get(module),
                        (Boolean)ReflectionUtils.getField(module.getClass(), "isParent").get(module),
                        (Boolean)ReflectionUtils.getField(module.getClass(), "isUserModule").get(module)
                );
                ReflectionUtils.deepCopy(module, photonLynxModule);
                map.remove(s, module);
                map.put(s, photonLynxModule);
                replacements.put(module, photonLynxModule);

                if(module.isParent() && LynxConstants.isEmbeddedSerialNumber(module.getSerialNumber())){
                    CONTROL_HUB = photonLynxModule;

                    LynxUsbDeviceImpl usbDevice;

                    ConcurrentHashMap<Integer, LynxRespondable> unfinishedCommands = new ConcurrentHashMap<>();
                    try {
                        Field f1 = module.getClass().getDeclaredField("lynxUsbDevice");
                        f1.setAccessible(true);
                        LynxUsbDevice tmp = (LynxUsbDevice) f1.get(module);
                        if(tmp instanceof LynxUsbDeviceDelegate){
                            Field tmp2 = LynxUsbDeviceDelegate.class.getDeclaredField("delegate");
                            tmp2.setAccessible(true);
                            usbDevice = (LynxUsbDeviceImpl) tmp2.get(tmp);
                        }else{
                            usbDevice = (LynxUsbDeviceImpl) tmp;
                        }
                        Field f2 = usbDevice.getClass().getSuperclass().getDeclaredField("robotUsbDevice");
                        f2.setAccessible(true);
                        Field f3 = usbDevice.getClass().getDeclaredField("engageLock");
                        f3.setAccessible(true);
                        syncLock = f3.get(usbDevice);

                        robotUsbDevice = (RobotUsbDevice) f2.get(usbDevice);
                    } catch (IllegalAccessException e) {
                        e.printStackTrace();
                    } catch(NoSuchFieldException e){
                        e.printStackTrace();
                    }
                }else{
                    EXPANSION_HUB = photonLynxModule;
                }
            } catch (IllegalAccessException e) {
                e.printStackTrace();
            } }

        HashMap<String, HardwareDevice> replacedNeutrino = new HashMap<>(), removedNeutrino = new HashMap<>();
        for(HardwareDevice device : map.getAll(HardwareDevice.class)){
            if(!(device instanceof LynxModule)){
                RobotLog.i(map.getNamesOf(device).toArray()[0].toString());
                if(device instanceof I2cDeviceSynchDevice){
                    try {
                        I2cDeviceSynchSimple device2 = (I2cDeviceSynchSimple) ReflectionUtils.getField(device.getClass(), "deviceClient").get(device);
                        if(!(device2 instanceof LynxI2cDeviceSynch)){
                            device2 = (I2cDeviceSynchSimple) ReflectionUtils.getField(device2.getClass(), "i2cDeviceSynchSimple").get(device2);
                        }
                        setLynxObject(device2, replacements);
                        RobotLog.e("" + (device2 instanceof LynxI2cDeviceSynch));
                    } catch (IllegalAccessException e) {
                        e.printStackTrace();
                    }
                }else if (device instanceof I2cDeviceSynchSimple){
                    try {
                        I2cDeviceSynchSimple device2 = (I2cDeviceSynchSimple) ReflectionUtils.getField(device.getClass(), "deviceClient").get(device);
                        setLynxObject(device2, replacements);
                    } catch (IllegalAccessException e) {
                        e.printStackTrace();
                    }
                }else {
                    setLynxObject(device, replacements);
                }
                if(device instanceof Rev2mDistanceSensor){
                    I2cDeviceSynch tmp = null;
                    try {
                        tmp = (I2cDeviceSynch) ReflectionUtils.getField(device.getClass(), "deviceClient").get(device);
                    } catch (IllegalAccessException e) {
                        e.printStackTrace();
                    }
                    Rev2mDistanceSensorEx vl53L0XEx = new Rev2mDistanceSensorEx(tmp);
                    replacedNeutrino.put((String) map.getNamesOf(device).toArray()[0], vl53L0XEx);
                    removedNeutrino.put((String) map.getNamesOf(device).toArray()[0], device);
                }
                if(device instanceof RevColorSensorV3){
                    RevColorSensorV3Ex revColorSensorV3Ex;
                    try {
                        I2cDeviceSynchSimple tmp = (I2cDeviceSynchSimple) ReflectionUtils.getField(device.getClass(), "deviceClient").get(device);
                        revColorSensorV3Ex = new RevColorSensorV3Ex(tmp);
                        replacedNeutrino.put((String) map.getNamesOf(device).toArray()[0], revColorSensorV3Ex);
                        removedNeutrino.put((String) map.getNamesOf(device).toArray()[0], device);
                    } catch (IllegalAccessException e) {
                        e.printStackTrace();
                    }
                }
            }
        }

        for(String s : replacedNeutrino.keySet()){
            map.remove(s, removedNeutrino.get(s));
            map.put(s, replacedNeutrino.get(s));
        }

        if(thisThread == null || !thisThread.isAlive()){
            thisThread = new Thread(this);
            threadEnabled.set(true);
            thisThread.start();
        }
    }

    private void setLynxObject(Object device, HashMap<LynxModule, PhotonLynxModule> replacements){
        Field f = ReflectionUtils.getField(device.getClass(), LynxModule.class);
        if (f != null) {
            f.setAccessible(true);
            try {
                LynxModule module = (LynxModule) f.get(device);
                if (module == null) {
                    return;
                }
                if (replacements.containsKey(module)) {
                    f.set(device, replacements.get(module));
                }
            } catch (IllegalAccessException e) {
                e.printStackTrace();
            }
        }
    }

    @Override
    public void onOpModePreStart(OpMode opMode) {

    }

    @Override
    public void onOpModePostStop(OpMode opMode) {
        enabled.set(false);
        threadEnabled.set(false);
    }
}

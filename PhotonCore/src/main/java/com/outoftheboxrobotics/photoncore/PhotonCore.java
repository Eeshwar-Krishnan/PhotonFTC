package com.outoftheboxrobotics.photoncore;

import android.content.Context;

import com.outoftheboxrobotics.photoncore.Commands.LynxStandardCommandV2;
import com.outoftheboxrobotics.photoncore.Reflection.ReflectionUtils;
import com.qualcomm.ftccommon.FtcEventLoop;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxUnsupportedCommandException;
import com.qualcomm.hardware.lynx.LynxUsbDevice;
import com.qualcomm.hardware.lynx.commands.LynxCommand;
import com.qualcomm.hardware.lynx.commands.LynxMessage;
import com.qualcomm.hardware.lynx.commands.core.LynxSetMotorConstantPowerCommand;
import com.qualcomm.hardware.lynx.commands.standard.LynxAck;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.ftccommon.external.OnCreateEventLoop;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.atomic.AtomicBoolean;

public class PhotonCore implements Runnable, OpModeManagerNotifier.Notifications {
    protected static final PhotonCore instance = new PhotonCore();
    private final ConcurrentHashMap<LynxCommand, Long> sentCommands;

    private final ConcurrentHashMap<LynxStandardCommandV2, LynxMessage> cachedResponses;
    private final ArrayList<CacheIntent> intents = new ArrayList<>();
    protected AtomicBoolean enabled = new AtomicBoolean(true), opmodeStopped = new AtomicBoolean(false);
    private List<LynxModule> modules;
    private Thread thisThread = null;
    private final Object waitObject = new Object(), sentLock = new Object(), intentLock = new Object();

    public static LynxModule CONTROL_HUB, EXPANSION_HUB;

    public PhotonCore(){
        sentCommands = new ConcurrentHashMap<>();
        cachedResponses = new ConcurrentHashMap<>();
        CONTROL_HUB = null;
        EXPANSION_HUB = null;
    }

    public static void addCacheIntent(CacheIntent intent){
        synchronized (instance.intentLock) {
            instance.intents.add(intent);
        }
    }

    @OnCreateEventLoop
    public static void attachEventLoop(Context context, FtcEventLoop eventLoop) {
        eventLoop.getOpModeManager().registerListener(instance);
    }

    protected static boolean registerSend(LynxCommand command) throws LynxUnsupportedCommandException, InterruptedException {
        if(true){
            return false;
        }
        long now = System.currentTimeMillis();

        if(command instanceof LynxSetMotorConstantPowerCommand){
            try {
                int motor = (byte) ReflectionUtils.getField(LynxSetMotorConstantPowerCommand.class, "motor").get(command);
                int power = (short) ReflectionUtils.getField(LynxSetMotorConstantPowerCommand.class, "power").get(command);
                synchronized (instance.sentCommands) {
                    while (instance.sentCommands.size() > 8);

                    LynxCommand command1 = new LynxSetMotorConstantPowerCommand(command.getModule(), motor, power);
                    instance.sentCommands.put(command1, System.currentTimeMillis());
                    command.onAckReceived(new LynxAck(command.getModule(), false));
                }
            } catch (IllegalAccessException e) {
                e.printStackTrace();
            }
        }

        return true;
    }

    protected static boolean shouldAckImmediately(LynxCommand command){
        return (command instanceof LynxSetMotorConstantPowerCommand);
    }

    public static void testSend(LynxCommand command){
        try {
            //command.getModule().acquireNetworkTransmissionLock(command);
            command.getModule().sendCommand(command);
            //command.getModule().releaseNetworkTransmissionLock(command);
        } catch (InterruptedException | LynxUnsupportedCommandException e) {
            e.printStackTrace();
        }
    }

    /**
     * if (otherCommand.getDestModuleAddress() == command.getDestModuleAddress() &&
     *                             otherCommand.getCommandNumber() == command.getCommandNumber() &&
     *                             Arrays.equals(otherCommand.toPayloadByteArray(), command.toPayloadByteArray()))
     *                         {
     * @param command
     * @return
     */
    protected static LynxMessage getCacheResponse(LynxCommand command){
        for(LynxStandardCommandV2 commandV2 : instance.cachedResponses.keySet()){
            if(command.getCommandNumber() == 4103){
            }
            if(commandV2.getDestModuleAddress() == command.getDestModuleAddress() &&
                commandV2.getCommandNumber() == command.getCommandNumber() &&
                    Arrays.equals(commandV2.toPayloadByteArray(), command.toPayloadByteArray())){
                //RobotLog.i("Bypassing Send: " + instance.cachedResponses.get(command));
                return instance.cachedResponses.get(commandV2);
            }
        }
        return null;
    }

    public static void disable(){
        instance.enabled.set(false);
    }

    public static void enable(){
        instance.enabled.set(true);
        synchronized (instance.waitObject) {
            instance.waitObject.notifyAll();
        }
    }

    @Override
    public void run() {
        ArrayList<LynxStandardCommandV2> sentCommands = new ArrayList<>();
        while(!instance.opmodeStopped.get()) {
            //RobotLog.i("***RUNNING LOOP***");
            while (!enabled.get()) {
                synchronized (instance.waitObject) {
                    try {
                        instance.waitObject.wait(1000);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }
            }

            synchronized (instance.sentCommands) {
                ArrayList<LynxCommand> removing = new ArrayList<>();
                for(LynxCommand command : instance.sentCommands.keySet()){
                    if(command.isAckOrResponseReceived()){
                        removing.add(command);
                    }else if((System.currentTimeMillis() - instance.sentCommands.get(command)) > 100){
                        removing.add(command);
                    }else if(!command.hasBeenTransmitted()){
                        try {
                            ((PhotonLynxModule)command.getModule()).normalSend(command);
                        } catch (LynxUnsupportedCommandException | InterruptedException e) {
                            e.printStackTrace();
                        }
                    }
                }

                for(LynxCommand command : removing){
                    instance.sentCommands.remove(command);
                }
            }

            ArrayList<LynxStandardCommandV2> toRemove = new ArrayList<>();
            for (LynxStandardCommandV2 command : sentCommands) {
                if (command.isResponded()) {
                    cachedResponses.put(command, command.getResponse());
                    toRemove.add(command);
                }
            }
            sentCommands.removeAll(toRemove);

            long now = System.nanoTime();
            ArrayList<LynxStandardCommandV2> toSend = new ArrayList<>();
            synchronized (intentLock) {
                for (CacheIntent intent : intents) {
                    if (intent.shouldRead()) {
                        toSend.add(intent.getCommand());
                    }
                }
            }

            for (LynxStandardCommandV2 commandv2 : toSend) {
                while (sentCommands.size() > 5) {
                    toRemove = new ArrayList<>();
                    for (LynxStandardCommandV2 command : sentCommands) {
                        if (command.isResponded()) {
                            cachedResponses.put(command, command.getResponse());
                            toRemove.add(command);
                        }
                    }
                    sentCommands.removeAll(toRemove);
                }
                try {
                    commandv2.getModule().sendCommand(commandv2);
                    sentCommands.add(commandv2);
                } catch (InterruptedException | LynxUnsupportedCommandException e) {
                    e.printStackTrace();
                }
            }

            while (sentCommands.size() > 0) {
                toRemove = new ArrayList<>();
                for (LynxStandardCommandV2 command : sentCommands) {
                    if (command.isResponded()) {
                        cachedResponses.put(command, command.getResponse());
                        toRemove.add(command);
                    }
                }
                sentCommands.removeAll(toRemove);
            }
        }
    }

    @Override
    public void onOpModePreInit(OpMode opMode) {
        instance.cachedResponses.clear();

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
            //RobotLog.ii("PHOTON", module.getClass()+" |");
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
                //RobotLog.i("*********REPLACED MODULE**********");
                if(module.isParent() && LynxConstants.isEmbeddedSerialNumber(module.getSerialNumber())){
                    CONTROL_HUB = photonLynxModule;
                }else{
                    EXPANSION_HUB = photonLynxModule;
                }
            } catch (IllegalAccessException e) {
                e.printStackTrace();
            } }
        for(HardwareDevice device : map.getAll(HardwareDevice.class)){
            //RobotLog.ii("PHOTON", device.getClass() + " | " + (device instanceof LynxModule));
            if(!(device instanceof LynxModule)){
                Field f = ReflectionUtils.getField(device.getClass(), "module");
                if(f != null){
                    f.setAccessible(true);
                    try {
                        LynxModule module = (LynxModule) f.get(device);
                        if(module == null){
                            continue;
                        }
                        if(!replacements.containsKey(module)){
                            //RobotLog.ii("No Replacement Found", "" + module.getClass());
                        }else {
                            f.set(device, replacements.get(module));
                        }
                    } catch (IllegalAccessException e) {
                        e.printStackTrace();
                    }
                }
            }
        }

        synchronized (intentLock) {
            instance.intents.clear();
        }

        opmodeStopped.set(false);
        enabled.set(false);
        if(thisThread == null || !thisThread.isAlive()){
            thisThread = new Thread(this);
            thisThread.start();
        }
    }

    @Override
    public void onOpModePreStart(OpMode opMode) {

    }

    @Override
    public void onOpModePostStop(OpMode opMode) {
        opmodeStopped.set(true);
        this.enabled.set(false);
    }
}

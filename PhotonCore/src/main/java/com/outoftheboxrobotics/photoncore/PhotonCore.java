package com.outoftheboxrobotics.photoncore;

import com.outoftheboxrobotics.photoncore.Commands.BulkData;
import com.outoftheboxrobotics.photoncore.Commands.LynxStandardCommandV2;
import com.outoftheboxrobotics.photoncore.Reflection.ReflectionUtils;
import com.qualcomm.hardware.lynx.LynxDcMotorController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.hardware.lynx.LynxUnsupportedCommandException;
import com.qualcomm.hardware.lynx.LynxUsbDevice;
import com.qualcomm.hardware.lynx.commands.LynxDatagram;
import com.qualcomm.hardware.lynx.commands.LynxMessage;
import com.qualcomm.hardware.lynx.commands.LynxRespondable;
import com.qualcomm.hardware.lynx.commands.LynxResponse;
import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataCommand;
import com.qualcomm.hardware.lynx.commands.standard.LynxStandardCommand;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.function.Predicate;

public class PhotonCore {
    private static final PhotonCore instance = new PhotonCore();
    private final ArrayList<LynxStandardCommandV2> sentCommands;
    private final HashMap<LynxStandardCommandV2, LynxMessage> cachedResponses;
    private final HashMap<LynxStandardCommandV2, Long> lastReads;
    private final ArrayList<LynxStandardCommandV2> readCommands, toBulkRead;
    private List<LynxModule> modules;

    public PhotonCore(){
        sentCommands = new ArrayList<>();
        lastReads = new HashMap<>();
        cachedResponses = new HashMap<>();
        readCommands = new ArrayList<>();
        toBulkRead = new ArrayList<>();
    }

    public static void setup(HardwareMap map){
        instance.modules = map.getAll(LynxModule.class);
        ArrayList<String> moduleNames = new ArrayList<>();
        for(LynxModule module : instance.modules){
            moduleNames.add((String) map.getNamesOf(module).toArray()[0]);
        }
        for(String s : moduleNames){
            LynxModule module = (LynxModule) map.get(s);
            try {
                PhotonLynxModule photonLynxModule = new PhotonLynxModule(
                        (LynxUsbDevice) ReflectionUtils.getField(LynxUsbDevice.class, "lynxUsbDevice").get(module),
                        (Integer)ReflectionUtils.getField(Integer.class, "moduleAddress").get(module),
                        (Boolean)ReflectionUtils.getField(Boolean.class, "isParent").get(module),
                        (Boolean)ReflectionUtils.getField(Boolean.class, "isUserModule").get(module)
                );
                map.remove(s, map.get(s));
                map.put(s, photonLynxModule);
            } catch (IllegalAccessException e) {
                e.printStackTrace();
            } }
    }

    protected static boolean registerSend(LynxStandardCommandV2 command){
        long timer = System.currentTimeMillis() + 100;
        while(instance.sentCommands.size() > 8){
            if(System.currentTimeMillis() > timer){
                return false;
            }
        }
        try {
            command.getModule().acquireNetworkTransmissionLock(command); //Get the network lock in order to send safely
            ((PhotonLynxModule)command.getModule()).normalSend(command);
            command.getModule().releaseNetworkTransmissionLock(command); //Don't need this anymore, will check for response later
            instance.sentCommands.add(command); //Keep track of this for ack/nack reasons
            return true;
        } catch (InterruptedException | LynxUnsupportedCommandException e) {
            e.printStackTrace();
            return true;
        }
    }

    protected static boolean registerGet(LynxStandardCommandV2 command, LynxStandardCommand userCommand) throws LynxNackException, InterruptedException, LynxUnsupportedCommandException {
        if(((LynxModule)userCommand.getModule()).getBulkCachingMode() == LynxModule.BulkCachingMode.OFF){
            ((PhotonLynxModule)command.getModule()).normalSend(userCommand);
        }

        long timer = System.currentTimeMillis() + 100;
        while(instance.sentCommands.size() > 8){
            if(System.currentTimeMillis() > timer){
                return false;
            }
        }

        LynxStandardCommandV2 key = getKey(command);
        if(key == null || !instance.toBulkRead.contains(key)){
            command.getModule().acquireNetworkTransmissionLock(userCommand); //Get the network lock in order to send safely
            ((PhotonLynxModule)command.getModule()).normalSend(userCommand);
            command.getModule().releaseNetworkTransmissionLock(userCommand); //Don't need this anymore, will check for response later
            instance.cachedResponses.put(command, command.getResponse());
            instance.lastReads.put(command, System.currentTimeMillis());
            instance.toBulkRead.add(command);
        }else{
            if(((LynxModule)userCommand.getModule()).getBulkCachingMode() == LynxModule.BulkCachingMode.AUTO) {
                if(alreadyRead(command)){
                    onBulkRead(null);
                }
            }
            instance.lastReads.put(command, System.currentTimeMillis());
            LynxMessage response = instance.cachedResponses.get(key);
            userCommand.onResponseReceived(response);
        }
        return true;
    }

    private static LynxStandardCommandV2 getKey(LynxStandardCommandV2 command){
        for(LynxStandardCommandV2 other : instance.cachedResponses.keySet()){
            if (other.getDestModuleAddress() == command.getDestModuleAddress() &&
                    other.getCommandNumber() == command.getCommandNumber() &&
                    Arrays.equals(other.toPayloadByteArray(), command.toPayloadByteArray())) {
                return other;
            }
        }
        return null;
    }

    private static boolean alreadyRead(LynxStandardCommandV2 command){
        for(LynxStandardCommandV2 other : instance.readCommands){
            if (other.getDestModuleAddress() == command.getDestModuleAddress() &&
                    other.getCommandNumber() == command.getCommandNumber() &&
                    Arrays.equals(other.toPayloadByteArray(), command.toPayloadByteArray())) {
                return true;
            }
        }
        return false;
    }

    protected static void onResponse(LynxStandardCommandV2 commandV2){
        if(instance.sentCommands.contains(commandV2)){
            instance.sentCommands.remove(commandV2);
        }else{
            RobotLog.w("Command response received with no record of sending");
        }
    }

    protected static void onBulkRead(LynxGetBulkInputDataCommand command) throws InterruptedException, LynxUnsupportedCommandException {
        if(instance.toBulkRead.size() == 0){
            return;
        }
        if(command == null){
            instance.modules.get(0).getBulkData();
            return;
        }
        instance.toBulkRead.removeIf(commandV2 -> (System.currentTimeMillis() - instance.lastReads.get(commandV2)) > 500);
        command.getModule().acquireNetworkTransmissionLock(command);
        for(LynxStandardCommandV2 commandV2 : instance.toBulkRead){
            ((PhotonLynxModule)commandV2.getModule()).normalSend(commandV2);
        }
        ((PhotonLynxModule)command.getModule()).normalSend(command);
        command.getModule().releaseNetworkTransmissionLock(command);

        boolean loop = true;
        while(loop){
            loop = false;
            for(LynxStandardCommandV2 commandV2 : instance.toBulkRead){
                if(!commandV2.isResponded()){
                    loop = true;
                }
            }
        }
    }
}

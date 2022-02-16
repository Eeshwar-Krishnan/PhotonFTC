package com.outoftheboxrobotics.photoncore;

import com.outoftheboxrobotics.photoncore.Commands.BulkData;
import com.outoftheboxrobotics.photoncore.Commands.LynxStandardCommandV2;
import com.outoftheboxrobotics.photoncore.Commands.V2.LynxGetBulkInputDataCommand;
import com.outoftheboxrobotics.photoncore.Reflection.ReflectionUtils;
import com.qualcomm.hardware.lynx.LynxDcMotorController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.hardware.lynx.LynxUnsupportedCommandException;
import com.qualcomm.hardware.lynx.commands.LynxMessage;
import com.qualcomm.hardware.lynx.commands.LynxResponse;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

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
    private final ArrayList<LynxStandardCommandV2> getCommands;
    private final HashMap<LynxStandardCommandV2, Long> lastRequest;

    private List<LynxModule> modules;
    private final HashMap<Integer, BulkData> bulkData;

    public PhotonCore(){
        sentCommands = new ArrayList<>();
        getCommands = new ArrayList<>();
        bulkData = new HashMap<>();
        lastRequest = new HashMap<>();
    }

    public static void setup(HardwareMap map){
        instance.modules = map.getAll(LynxModule.class);
        ArrayList<String> moduleNames = new ArrayList<>();
        for(LynxModule module : instance.modules){
            com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataCommand command = new com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataCommand(module);
            try {
                instance.bulkData.put(module.getModuleAddress(), new BulkData(command.sendReceive(), false));
            } catch (InterruptedException | LynxNackException e) {
                e.printStackTrace();
            }
            moduleNames.add((String) map.getNamesOf(module).toArray()[0]);
        }
        for(String s : moduleNames){
            LynxModule module = (LynxModule) map.get(s);
            try {
                PhotonLynxModule photonLynxModule = new PhotonLynxModule(
                ReflectionUtils.getField(LynxModule.class, "lynxUsbDevice").get(module),
                ReflectionUtils.getField(LynxModule.class, "moduleAddress").get(module),
                ReflectionUtils.getField(LynxModule.class, "isParent").get(module),
                ReflectionUtils.getField(LynxModule.class, "isUserModule").get(module)
                );
                map.remove(s, map.get(s));
                map.put(s, photonLynxModule);
            } catch (IllegalAccessException e) {
                e.printStackTrace();
            } }

        for(DcMotorEx motor : map.getAll(DcMotorEx.class)){
            String name = (String) map.getNamesOf(motor).toArray()[0];

            try {
                LynxDcMotorController controller = (LynxDcMotorController) ReflectionUtils.getField(DcMotorEx.class, "controllerEx").get(motor);
                LynxModule module = (LynxModule) ReflectionUtils.getField(DcMotorEx.class, "module").get(motor);
                int portNum = (int) ReflectionUtils.getField(DcMotorEx.class, "portNumber").get(motor);

                map.remove(name, motor);
                map.put(name, new PhotonMotor(module, controller, portNum));
            } catch (IllegalAccessException e) {
                e.printStackTrace();
            }
        }
    }

    protected static void registerSend(LynxStandardCommandV2 command){
        try {
            command.getModule().acquireNetworkTransmissionLock(command); //Get the network lock in order to send safely
            command.getModule().sendCommand(command); //Send out the command!
            command.getModule().releaseNetworkTransmissionLock(command); //Don't need this anymore, will check for response later
            instance.sentCommands.add(command); //Keep track of this for ack/nack reasons
        } catch (InterruptedException | LynxUnsupportedCommandException e) {
            e.printStackTrace();
        }
    }

    protected static LynxMessage registerGet(LynxStandardCommandV2 command){
        for(LynxStandardCommandV2 otherCommand : instance.getCommands){
            if (otherCommand.getDestModuleAddress() == command.getDestModuleAddress() &&
                    otherCommand.getCommandNumber() == command.getCommandNumber() &&
                    Arrays.equals(otherCommand.toPayloadByteArray(), command.toPayloadByteArray())) {
                instance.lastRequest.put(otherCommand, System.currentTimeMillis());
                return otherCommand.getResponse();
            }
        }
        try {
            command.getModule().acquireNetworkTransmissionLock(command); //Get the network lock in order to send safely
            command.getModule().sendCommand(command); //Send out the command!
            command.getModule().releaseNetworkTransmissionLock(command); //Don't need this anymore, will check for response later

            long timer = System.currentTimeMillis() + 25;
            while(!command.isResponded() && timer > System.currentTimeMillis());

            instance.getCommands.add(command); //Keep track of this so we can send it regularly
            instance.lastRequest.put(command, System.currentTimeMillis());

            return command.getResponse();
        } catch (InterruptedException | LynxUnsupportedCommandException e) {
            e.printStackTrace();
        }

        return null;
    }

    protected static BulkData getBulkData(int moduleAddress){
        synchronized (instance.bulkData) {
            return instance.bulkData.get(moduleAddress);
        }
    }

    public static void update(){
        HashMap<LynxModule, LynxGetBulkInputDataCommand> bulkCommands = new HashMap<>();

        //Send off all the get commands now so they process while we wait for the set commands to finish
        for(LynxModule module : instance.modules){
            LynxGetBulkInputDataCommand bulkCommand = new LynxGetBulkInputDataCommand(module);
            try {
                module.acquireNetworkTransmissionLock(bulkCommand);
                module.sendCommand(bulkCommand);
                bulkCommands.put(module, bulkCommand);
            } catch (InterruptedException | LynxUnsupportedCommandException e) {
                e.printStackTrace();
            }
            LynxStandardCommandV2 acquiredCommand = null;
            for(LynxStandardCommandV2 command : instance.getCommands){
                if(command.getModuleAddress() == module.getModuleAddress()){
                    try {
                        module.sendCommand(command);
                    } catch (InterruptedException | LynxUnsupportedCommandException e) {
                        e.printStackTrace();
                    }
                }
            }
            try {
                module.releaseNetworkTransmissionLock(bulkCommand);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        //Make sure to resend any command that was nacked
        ArrayList<LynxStandardCommandV2> resendCommands = new ArrayList<>();
        while(instance.sentCommands.size() > 0){
            for(LynxStandardCommandV2 command : instance.sentCommands){
                if(command.isResponded()){
                    if(command.isNacked()){
                        command.reset();
                        resendCommands.add(command);
                    }
                    instance.sentCommands.remove(command);
                }
            }
        }

        for (LynxStandardCommandV2 command : resendCommands) {
            instance.registerSend(command);
        }

        //Wait for everything to be responded to so that we can get all the data we need for the next loop
        boolean end = false;
        while(!end){
            end = true;
            for(LynxStandardCommandV2 command : instance.getCommands){
                if(!command.isResponded()){
                    end = false;
                }
            }
        }

        for(LynxStandardCommandV2 command : instance.getCommands){
            command.reset();
        }

        //Finally grab the bulk data
        for(LynxModule module : bulkCommands.keySet()){
            while(!bulkCommands.get(module).isResponded());
            synchronized (instance.bulkData) {
                instance.bulkData.put(module.getModuleAddress(), new BulkData(bulkCommands.get(module).getResponse(), false));
            }
        }

        //Prune the bulk requests that haven't been used for over a second
        long now = System.currentTimeMillis();
        instance.getCommands.removeIf(lynxStandardCommandV2 -> (now - instance.lastRequest.get(lynxStandardCommandV2)) > 1000);
    }
}

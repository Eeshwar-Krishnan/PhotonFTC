package com.outoftheboxrobotics.photoncore;

import com.outoftheboxrobotics.photoncore.Commands.LynxStandardCommandV2;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.hardware.lynx.LynxUnsupportedCommandException;
import com.qualcomm.hardware.lynx.LynxUsbDevice;
import com.qualcomm.hardware.lynx.commands.LynxDatagram;
import com.qualcomm.hardware.lynx.commands.LynxMessage;
import com.qualcomm.hardware.lynx.commands.LynxRespondable;
import com.qualcomm.hardware.lynx.commands.core.*;
import com.qualcomm.hardware.lynx.commands.standard.LynxAck;
import com.qualcomm.hardware.lynx.commands.standard.LynxStandardCommand;

import org.apache.commons.beanutils.PropertyUtils;
import org.reflections.Reflections;

import java.lang.reflect.InvocationTargetException;
import java.util.Iterator;
import java.util.Set;

import static org.reflections.scanners.Scanners.SubTypes;

public class PhotonLynxModule extends LynxModule {
    Set<Class<? extends LynxStandardCommandV2>> v2Classes;

    public PhotonLynxModule(LynxUsbDevice lynxUsbDevice, int moduleAddress, boolean isParent, boolean isUserModule) {
        super(lynxUsbDevice, moduleAddress, isParent, isUserModule);
        Reflections reflection = new Reflections("com.outoftheboxrobotics.photoncore.Commands.V2");
        v2Classes = reflection.getSubTypesOf(LynxStandardCommandV2.class);
    }

    @Override
    public void sendCommand(LynxMessage command) throws InterruptedException, LynxUnsupportedCommandException {
        if(isSetCommand(command)){
            try {
                LynxStandardCommandV2 commandv2 = exchangeCommand(command);
                if(commandv2 == null){
                    super.sendCommand(command);
                }else {
                    boolean sent = PhotonCore.registerSend(commandv2);
                    if(!sent){
                        super.sendCommand(command);
                    }
                    if(command instanceof LynxRespondable) {
                        LynxAck ack = new LynxAck(this, false);
                        ((LynxRespondable)command).onAckReceived(ack);
                    }
                }
            } catch (InstantiationException | IllegalAccessException | InvocationTargetException | NoSuchMethodException e) {
                e.printStackTrace();
            }
        }else if(isValidGetCommand(command)){
            if(command instanceof LynxGetBulkInputDataCommand){
                PhotonCore.onBulkRead((LynxGetBulkInputDataCommand)command);
            }else{
                try {
                    LynxStandardCommandV2 commandv2 = exchangeCommand(command);
                    if(commandv2 == null){
                        super.sendCommand(command);
                    }
                    boolean sent = PhotonCore.registerGet(commandv2, (LynxStandardCommand) command);
                    if(!sent){
                        super.sendCommand(command);
                    }
                } catch (InstantiationException | IllegalAccessException | InvocationTargetException | NoSuchMethodException | LynxNackException e) {
                    e.printStackTrace();
                }
            }
        }else{
            super.sendCommand(command);
        }
    }

    public void normalSend(LynxMessage command) throws LynxUnsupportedCommandException, InterruptedException {
        super.sendCommand(command);
    }

    private boolean isSetCommand(LynxMessage command){
        return command instanceof LynxSetAllDIOOutputsCommand ||
                command instanceof LynxSetDIODirectionCommand ||
                command instanceof LynxSetMotorChannelCurrentAlertLevelCommand ||
                command instanceof LynxSetMotorChannelEnableCommand ||
                command instanceof LynxSetMotorChannelModeCommand ||
                command instanceof LynxSetMotorConstantPowerCommand ||
                command instanceof LynxSetMotorPIDControlLoopCoefficientsCommand ||
                command instanceof LynxSetMotorPIDFControlLoopCoefficientsCommand ||
                command instanceof LynxSetMotorTargetPositionCommand ||
                command instanceof LynxSetMotorTargetVelocityCommand ||
                command instanceof LynxSetPWMConfigurationCommand ||
                command instanceof LynxSetPWMEnableCommand ||
                command instanceof LynxSetPWMPulseWidthCommand ||
                command instanceof LynxSetServoConfigurationCommand ||
                command instanceof LynxSetServoEnableCommand ||
                command instanceof LynxSetServoPulseWidthCommand ||
                command instanceof LynxSetSingleDIOOutputCommand ||
                command instanceof LynxI2cConfigureChannelCommand ||
                command instanceof LynxI2cReadMultipleBytesCommand ||
                command instanceof LynxI2cReadSingleByteCommand ||
                command instanceof LynxI2cWriteMultipleBytesCommand ||
                command instanceof LynxI2cWriteSingleByteCommand ||
                command instanceof LynxI2cWriteReadMultipleBytesCommand ||
                command instanceof LynxResetMotorEncoderCommand;
    }

    private boolean isValidGetCommand(LynxMessage command){
        return (!isSetCommand(command)) &&
                !(command instanceof LynxI2cReadStatusQueryCommand) &&
                !(command instanceof LynxI2cWriteStatusQueryCommand);
    }

    private boolean isGetCommand(LynxMessage command){
        return !isSetCommand(command);
    }

    private LynxStandardCommandV2 exchangeCommand(LynxMessage command) throws InstantiationException, IllegalAccessException, InvocationTargetException, NoSuchMethodException {
        Iterator<Class<? extends LynxStandardCommandV2>> iterator = v2Classes.iterator();
        while(iterator.hasNext()){
            Class<? extends LynxStandardCommandV2> clazz = iterator.next();
            if(clazz.getName().equals(command)){
                LynxStandardCommandV2 commandv2 = clazz.newInstance();
                PropertyUtils.copyProperties(command, commandv2);
                return commandv2;
            }
        }
        return null;
    }
}

package com.outoftheboxrobotics.photoncore.HAL;

import com.outoftheboxrobotics.photoncore.HAL.Motors.Commands.PhotonCommandBase;
import com.outoftheboxrobotics.photoncore.HAL.Motors.PhotonDcMotor;
import com.outoftheboxrobotics.photoncore.HAL.Motors.PhotonLynxDCMotorController;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.hardware.lynx.LynxUnsupportedCommandException;
import com.qualcomm.hardware.lynx.LynxUsbDevice;
import com.qualcomm.hardware.lynx.commands.LynxMessage;
import com.qualcomm.hardware.lynx.commands.LynxRespondable;
import com.qualcomm.hardware.lynx.commands.LynxResponse;
import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataResponse;

import java.util.HashMap;
import java.util.LinkedList;
import java.util.concurrent.CompletableFuture;

public class PhotonHAL implements HAL{
    private final LynxModule lynxModule;
    private final Object theLock = new Object();

    private PhotonLynxDCMotorController controller;

    public PhotonHAL(LynxModule lynxModule){
        this.lynxModule = lynxModule;
        controller = new PhotonLynxDCMotorController(this);
    }

    @Override
    public void write(LynxRespondable respondable) {
        synchronized (theLock){
            try {
                lynxModule.sendCommand(respondable);
            } catch (InterruptedException | LynxUnsupportedCommandException e) {
                e.printStackTrace();
            }
        }
    }

    public PhotonDcMotor getMotor(int port){
        return controller.getMotor(port);
    }

    public LynxModule getLynxModule() {
        return lynxModule;
    }

    public LynxGetBulkInputDataResponse getBulkData(){
        return PhotonCore.getControlData();
    }
}

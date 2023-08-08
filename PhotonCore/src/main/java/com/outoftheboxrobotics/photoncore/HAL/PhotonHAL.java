package com.outoftheboxrobotics.photoncore.HAL;

import com.outoftheboxrobotics.photoncore.HAL.Motors.Commands.PhotonCommandBase;
import com.outoftheboxrobotics.photoncore.HAL.Motors.Commands.PhotonLynxGetBulkInputDataCommand;
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
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.HashMap;
import java.util.LinkedList;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.atomic.AtomicBoolean;

public class PhotonHAL implements HAL{
    private final LynxModule lynxModule;
    private final Object theLock = new Object();
    private final Object commandLock = new Object();
    private LynxGetBulkInputDataResponse commandBulkData = null;
    private final AtomicBoolean controlInFlight = new AtomicBoolean(false);

    private final PhotonLynxDCMotorController controller;

    public PhotonHAL(LynxModule lynxModule){
        this.lynxModule = lynxModule;
        controller = new PhotonLynxDCMotorController(this);
    }

    @Override
    public void write(LynxRespondable respondable) {
        synchronized (theLock){
            try {
                PhotonCore.submit((PhotonCommandBase)respondable);
                lynxModule.sendCommand(respondable);
            } catch (InterruptedException | LynxUnsupportedCommandException e) {
                e.printStackTrace();
            }
        }
    }

    @Override
    public void refreshCache(){
        if(controlInFlight.get())
            return;
        PhotonLynxGetBulkInputDataCommand command = new PhotonLynxGetBulkInputDataCommand(getLynxModule());
        try {
            command.getResponse().thenApply((response) -> {
                synchronized (commandLock) {
                    commandBulkData = (LynxGetBulkInputDataResponse) response;
                    controlInFlight.set(false);
                }
                return 1;
            });
            controlInFlight.set(true);
            write(command);
        } catch (LynxNackException e) {
            e.printStackTrace();
        }
    }

    @Override
    public DcMotorEx getMotor(int port){
        return controller.getMotor(port);
    }

    @Override
    public LynxModule getLynxModule() {
        return lynxModule;
    }

    @Override
    public LynxGetBulkInputDataResponse getBulkData(){
        return PhotonCore.getControlData();
    }
}

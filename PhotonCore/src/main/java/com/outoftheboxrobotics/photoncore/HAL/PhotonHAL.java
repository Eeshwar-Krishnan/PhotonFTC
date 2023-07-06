package com.outoftheboxrobotics.photoncore.HAL;

import com.outoftheboxrobotics.photoncore.HAL.Motors.Commands.PhotonCommandBase;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.hardware.lynx.LynxUnsupportedCommandException;
import com.qualcomm.hardware.lynx.LynxUsbDevice;
import com.qualcomm.hardware.lynx.commands.LynxMessage;
import com.qualcomm.hardware.lynx.commands.LynxRespondable;
import com.qualcomm.hardware.lynx.commands.LynxResponse;
import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataResponse;

import java.util.LinkedList;
import java.util.concurrent.CompletableFuture;

public class PhotonHAL implements HAL{
    private LynxModule lynxModule;
    private final Object theLock = new Object();

    private final LinkedList<Integer> numbersQueue = new LinkedList<Integer>();

    public PhotonHAL(LynxModule lynxModule){
        this.lynxModule = lynxModule;

        for(int b = 0; b < 255; b ++){
            numbersQueue.add(b+1);
        }
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

    public LynxModule getLynxModule() {
        return lynxModule;
    }

    public LynxGetBulkInputDataResponse getBulkData(){
        return null;
    }
}

package com.outoftheboxrobotics.photoncore;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxUnsupportedCommandException;
import com.qualcomm.hardware.lynx.LynxUsbDevice;
import com.qualcomm.hardware.lynx.commands.LynxCommand;
import com.qualcomm.hardware.lynx.commands.LynxMessage;
import com.qualcomm.hardware.lynx.commands.LynxRespondable;
import com.qualcomm.hardware.lynx.commands.standard.LynxAck;

import java.util.ArrayList;
import java.util.concurrent.ConcurrentHashMap;

public class PhotonLynxModule extends LynxModule {
    private ArrayList<LynxMessage> skippedAcquire = new ArrayList<>();

    public PhotonLynxModule(LynxUsbDevice lynxUsbDevice, int moduleAddress, boolean isParent, boolean isUserModule) {
        super(lynxUsbDevice, moduleAddress, isParent, isUserModule);
    }

    public ConcurrentHashMap<Integer, LynxRespondable> getUnfinishedCommands(){
        return this.unfinishedCommands;
    }

    @Override
    protected byte getNewMessageNumber() {
        return super.getNewMessageNumber();
    }

    @Override
    public void sendCommand(LynxMessage command) throws InterruptedException, LynxUnsupportedCommandException {
        if(!PhotonCore.instance.enabled.get()){
            super.sendCommand(command);
            return;
        }
        if(command instanceof LynxCommand){
            if(PhotonCore.getCacheResponse((LynxCommand)command) != null){
                ((LynxCommand) command).onResponseReceived(PhotonCore.getCacheResponse((LynxCommand)command));
                return;
            }
            if(PhotonCore.shouldParallelize((LynxCommand)command)){
                boolean success = PhotonCore.registerSend((LynxCommand) command);
                if (!success) {
                    super.sendCommand(command);
                }
                return;
            }
        }
        super.sendCommand(command);
    }

    @Override
    public void acquireNetworkTransmissionLock(LynxMessage message) throws InterruptedException {
        if(!PhotonCore.instance.enabled.get()){
            super.acquireNetworkTransmissionLock(message);
            return;
        }
        if(message instanceof LynxCommand){
            if(PhotonCore.getCacheResponse((LynxCommand)message) != null){
                skippedAcquire.add(message);
                return;
            }
            if(PhotonCore.shouldParallelize((LynxCommand) message)){
                skippedAcquire.add(message);
                return;
            }
        }
        super.acquireNetworkTransmissionLock(message);
    }

    @Override
    public void releaseNetworkTransmissionLock(LynxMessage message) throws InterruptedException {
        if(!PhotonCore.instance.enabled.get()){
            super.releaseNetworkTransmissionLock(message);
            return;
        }
        if(skippedAcquire.contains(message)){
            skippedAcquire.remove(message);
            return;
        }
        super.releaseNetworkTransmissionLock(message);
    }
}

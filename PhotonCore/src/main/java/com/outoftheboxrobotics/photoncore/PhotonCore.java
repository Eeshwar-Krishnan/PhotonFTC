package com.outoftheboxrobotics.photoncore;

import android.content.Context;

import com.qualcomm.ftccommon.FtcEventLoop;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.commands.LynxRespondable;
import com.qualcomm.hardware.lynx.commands.LynxResponse;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerImpl;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier;
import com.qualcomm.robotcore.hardware.usb.RobotUsbDevice;

import org.firstinspires.ftc.ftccommon.external.OnCreateEventLoop;

import java.util.HashMap;
import java.util.List;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;

public class PhotonCore implements Runnable, OpModeManagerNotifier.Notifications {
    protected static final PhotonCore instance = new PhotonCore();
    protected AtomicBoolean enabled, threadEnabled;

    private HashMap<LynxRespondable, CompletableFuture<LynxResponse>> commandMap;

    private OpModeManagerImpl opModeManager;

    public static class ExperimentalParameters{
        private final AtomicBoolean singlethreadedOptimized = new AtomicBoolean(true);
        private final AtomicInteger maximumParallelCommands = new AtomicInteger(4);

        public void setSinglethreadedOptimized(boolean state){
            this.singlethreadedOptimized.set(state);
        }

        public boolean setMaximumParallelCommands(int maximumParallelCommands){
            if(maximumParallelCommands <= 0){
                return false;
            }
            this.maximumParallelCommands.set(maximumParallelCommands);
            return true;
        }
    }

    public static ExperimentalParameters experimental = new ExperimentalParameters();

    public PhotonCore(){
        enabled = new AtomicBoolean(false);
        threadEnabled = new AtomicBoolean(false);
        commandMap = new HashMap<>();
    }

    public static void enable(){
        instance.enabled.set(true);
    }

    public static void disable(){
        instance.enabled.set(false);
    }

    @OnCreateEventLoop
    public static void attachEventLoop(Context context, FtcEventLoop eventLoop) {
        eventLoop.getOpModeManager().registerListener(instance);
        instance.opModeManager = eventLoop.getOpModeManager();
    }

    @Override
    public void run() {
        while(threadEnabled.get()){
            if(enabled.get()){
                for(LynxRespondable respondable : commandMap.keySet()){
                    if(respondable.hasBeenAcknowledged()){
                        
                    }
                }
            }
            try {
                Thread.sleep(5);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

    @Override
    public void onOpModePreInit(OpMode opMode) {

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

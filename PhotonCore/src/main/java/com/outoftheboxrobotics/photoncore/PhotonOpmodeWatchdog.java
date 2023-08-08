package com.outoftheboxrobotics.photoncore;

import android.content.Context;

import com.qualcomm.ftccommon.FtcEventLoop;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier;

import org.firstinspires.ftc.ftccommon.external.OnCreateEventLoop;

public class PhotonOpmodeWatchdog implements OpModeManagerNotifier.Notifications {
    private static PhotonOpmodeWatchdog instance = null;
    private int state = 0;

    public static PhotonOpmodeWatchdog getInstance(){
        if(instance == null){
            instance = new PhotonOpmodeWatchdog();
        }
        return instance;
    }

    public static int getState(){
        return getInstance().state;
    }

    @OnCreateEventLoop
    public static void attachEventLoop(Context context, FtcEventLoop eventLoop) {
        eventLoop.getOpModeManager().registerListener(getInstance());
    }

    @Override
    public void onOpModePreInit(OpMode opMode) {
        state = 0;
    }

    @Override
    public void onOpModePreStart(OpMode opMode) {
        state = 1;
    }

    @Override
    public void onOpModePostStop(OpMode opMode) {
        state = 2;
    }
}

package com.outoftheboxrobotics.photoncore;

import android.content.Context;
import android.os.Environment;

import com.outoftheboxrobotics.photoncore.hardware.PhotonLynxCommandListener;
import com.qualcomm.ftccommon.FtcEventLoop;
import com.qualcomm.hardware.lynx.commands.LynxCommand;
import com.qualcomm.hardware.lynx.commands.LynxMessage;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerImpl;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.ftccommon.external.OnCreateEventLoop;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Locale;


public class PhotonProfiler implements OpModeManagerNotifier.Notifications, PhotonLynxCommandListener {
    private static final PhotonProfiler instance = new PhotonProfiler();
    private static final String TAG = "PhotonProfiler";

    private static boolean attached = false;
    private static File logFile;
    private static FileWriter logFileWriter;
    private OpModeManagerImpl opModeManager;

    @SuppressWarnings({"unused"})
    @OnCreateEventLoop
    public static void attachEventLoop(Context context, FtcEventLoop eventLoop)
    {
        instance.opModeManager=eventLoop.getOpModeManager();
    }

    public static void attach()
    {
        attached=PhotonCore.addLynxCommandListener(instance);
        if(PhotonCore.DEBUG)
        {
            if(attached) RobotLog.i(TAG, "attach(): attached PhotonProfiler");
            else RobotLog.e(TAG, "attach(): failed attaching PhotonProfiler");
        }

        try {
            logFile=new File(String.format("%s/FIRST/%s", Environment.getExternalStorageDirectory(), instance.opModeManager.getActiveOpModeName()+".log"));
            if(!logFile.exists()) assert(logFile.createNewFile());
            logFileWriter=new FileWriter(logFile);
            instance.logEvent(EventType.ATTACHED);
        } catch (IOException e) {
            RobotLog.e(TAG, "attach(): failed opening file");
            attached=false;
        }
    }
    public static void annotateLoop()
    {
        instance.logEvent(EventType.LOOP);
    }

    private synchronized void logEvent(EventType event) {
        if(attached)
        {
            try {
                logFileWriter.write(String.format(Locale.ENGLISH, "%d %s\n", System.nanoTime(), event.toString()));
            } catch (IOException e) {
                RobotLog.ee(TAG, "logEvent(): failed logging event %s", event.toString());
            }
        }
    }
    private synchronized void logCommand(LynxCommand lynxCommand) {
        if(attached)
        {
            try {
                logFileWriter.write(String.format(Locale.ENGLISH, "%d %s\n", System.nanoTime(), lynxCommand.toString()));
            } catch (IOException e) {
                RobotLog.ee(TAG, "logEvent(): failed logging command %s", lynxCommand.toString());
            }
        }
    }

    private synchronized void logResponseReceived(LynxMessage response, LynxCommand respondedCommand)
    {
        if(attached)
        {
            try {
                logFileWriter.write(String.format(Locale.ENGLISH, "%d %s %s\n", System.nanoTime(), respondedCommand.toString(), response.toString()));
            } catch (IOException e) {
                RobotLog.ee(TAG, "logEvent(): failed logging response %s for command %s", response.toString(), respondedCommand.toString());
            }
        }
    }


    @Override
    public void onOpModePreInit(OpMode opMode) {
        logEvent(EventType.PRE_INIT);
    }

    @Override
    public void onOpModePreStart(OpMode opMode) {
        logEvent(EventType.PRE_START);
    }

    @Override
    public void onOpModePostStop(OpMode opMode) {
        logEvent(EventType.PRE_STOP);
    }

    @Override
    public void onCommand(LynxCommand command) {
        logCommand(command);
    }

    @Override
    public void onCommandResponse(LynxMessage response, LynxCommand respondedCommand) {
        logResponseReceived(response, respondedCommand);
    }

    enum EventType {
        PRE_INIT("PRE_INIT"), ATTACHED("ATTACHED"), PRE_START("START"), PRE_STOP("STOP"), LOOP("LOOP");
        EventType(String identifier)
        {
            this.identifier=identifier;
        }
        private final String identifier;
        public String toString()
        {
            return  identifier;
        }
    }
}

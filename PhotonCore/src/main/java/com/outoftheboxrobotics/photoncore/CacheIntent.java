package com.outoftheboxrobotics.photoncore;

import com.qualcomm.robotcore.util.ElapsedTime;

import com.outoftheboxrobotics.photoncore.Commands.LynxStandardCommandV2;

public abstract class CacheIntent {
    private long runDelayMs;
    private ElapsedTime timer;
    public CacheIntent(long runDelayMs){
        this.runDelayMs = runDelayMs;
        timer = new ElapsedTime();
    }

    protected abstract LynxStandardCommandV2 getCommand();

    protected long getFrequency(){
        return runDelayMs;
    }

    protected boolean shouldRead(){
        if(timer.milliseconds() >= runDelayMs){
            timer.reset();
            return true;
        }
        return false;
    }
}

package com.outoftheboxrobotics.photoncore.Caching;

import com.qualcomm.hardware.lynx.LynxModule;

import com.outoftheboxrobotics.photoncore.CacheIntent;
import com.outoftheboxrobotics.photoncore.Commands.LynxStandardCommandV2;
import com.outoftheboxrobotics.photoncore.Commands.V2.*;

public class ModuleCurrentCacheIntent extends CacheIntent {
    private LynxModule module;
    private LynxStandardCommandV2 commandV2;

    public ModuleCurrentCacheIntent(long runDelayMs, LynxModule module) {
        super(runDelayMs);
        this.module = module;
        this.commandV2 = new LynxGetADCCommand(module, LynxGetADCCommand.Channel.BATTERY_CURRENT, LynxGetADCCommand.Mode.ENGINEERING);
    }

    @Override
    protected LynxStandardCommandV2 getCommand() {
        return commandV2;
    }
}

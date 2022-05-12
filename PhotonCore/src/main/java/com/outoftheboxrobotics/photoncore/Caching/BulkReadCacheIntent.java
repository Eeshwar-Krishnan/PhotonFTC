package com.outoftheboxrobotics.photoncore.Caching;

import com.qualcomm.hardware.lynx.LynxModule;

import com.outoftheboxrobotics.photoncore.CacheIntent;
import com.outoftheboxrobotics.photoncore.Commands.LynxStandardCommandV2;
import com.outoftheboxrobotics.photoncore.Commands.V2.LynxGetBulkInputDataCommand;

public class BulkReadCacheIntent extends CacheIntent {
    private LynxModule module;
    private LynxStandardCommandV2 commandV2;

    public BulkReadCacheIntent(long runDelayMs, LynxModule module) {
        super(runDelayMs);
        this.module = module;
        this.commandV2 = new LynxGetBulkInputDataCommand(module);
    }

    @Override
    protected LynxStandardCommandV2 getCommand() {
        return commandV2;
    }
}

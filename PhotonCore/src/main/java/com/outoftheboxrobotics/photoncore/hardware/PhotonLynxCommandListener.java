package com.outoftheboxrobotics.photoncore.hardware;

import com.qualcomm.hardware.lynx.commands.LynxCommand;
import com.qualcomm.hardware.lynx.commands.LynxMessage;

public interface PhotonLynxCommandListener {
    void onCommand(LynxCommand command);
    void onCommandResponse(LynxMessage response, LynxCommand respondedCommand);
}

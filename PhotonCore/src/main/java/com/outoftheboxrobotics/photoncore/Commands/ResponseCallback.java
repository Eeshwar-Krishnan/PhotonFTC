package com.outoftheboxrobotics.photoncore.Commands;

import com.qualcomm.hardware.lynx.commands.LynxMessage;
import com.qualcomm.hardware.lynx.commands.LynxResponse;

public interface ResponseCallback {
    void call(LynxMessage response);
}

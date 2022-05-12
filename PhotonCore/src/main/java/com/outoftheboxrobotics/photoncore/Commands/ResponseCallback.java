package com.outoftheboxrobotics.photoncore.Commands;

import com.qualcomm.hardware.lynx.commands.LynxMessage;

public interface ResponseCallback {
    void call(LynxMessage response);
}

package com.outoftheboxrobotics.photoncore.HAL.I2C;

import com.qualcomm.hardware.lynx.commands.LynxCommand;

public interface PhotonLynxI2cDeviceSynch {
    void setInterleavedCommand(LynxCommand<?> command);
    void clearInterleavedCommand();
}

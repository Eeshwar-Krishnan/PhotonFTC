package com.outoftheboxrobotics.photoncore.HAL;

import com.outoftheboxrobotics.photoncore.HAL.Motors.Commands.PhotonCommandBase;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.hardware.lynx.commands.LynxMessage;
import com.qualcomm.hardware.lynx.commands.LynxRespondable;
import com.qualcomm.hardware.lynx.commands.LynxResponse;
import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataResponse;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.concurrent.CompletableFuture;

public interface HAL {
    void write(LynxRespondable respondable);

    LynxModule getLynxModule();

    LynxGetBulkInputDataResponse getBulkData();

    void refreshCache();

    DcMotorEx getMotor(int port);
}

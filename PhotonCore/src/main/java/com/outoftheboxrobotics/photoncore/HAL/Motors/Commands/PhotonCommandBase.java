package com.outoftheboxrobotics.photoncore.HAL.Motors.Commands;

import com.qualcomm.hardware.lynx.LynxModuleIntf;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.hardware.lynx.commands.LynxMessage;
import com.qualcomm.hardware.lynx.commands.LynxRespondable;
import com.qualcomm.hardware.lynx.commands.LynxResponse;
import com.qualcomm.hardware.lynx.commands.core.LynxDekaInterfaceCommand;
import com.qualcomm.hardware.lynx.commands.standard.LynxAck;
import com.qualcomm.hardware.lynx.commands.standard.LynxNack;

import java.util.concurrent.CompletableFuture;

public interface PhotonCommandBase{
    CompletableFuture<LynxMessage> getResponse() throws LynxNackException;
}

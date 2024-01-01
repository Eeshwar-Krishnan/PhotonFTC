package com.outoftheboxrobotics.photoncore.hardware.servo;

import android.content.Context;

import com.outoftheboxrobotics.photoncore.hardware.servo.commands.PhotonLynxGetServoEnableCommand;
import com.outoftheboxrobotics.photoncore.hardware.servo.commands.PhotonLynxGetServoPulseWidthCommand;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.hardware.lynx.LynxServoController;
import com.qualcomm.hardware.lynx.LynxUsbUtil;
import com.qualcomm.hardware.lynx.commands.core.LynxGetServoEnableResponse;
import com.qualcomm.hardware.lynx.commands.core.LynxGetServoPulseWidthResponse;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.util.Range;

import java.util.concurrent.CompletableFuture;

public class PhotonLynxServoController extends LynxServoController implements PhotonServoController{

    public PhotonLynxServoController(Context context, LynxModule module) throws RobotCoreException, InterruptedException {
        super(context, module);
    }

    @Override
    public CompletableFuture<Boolean> isServoPwmEnabledAsync(int servo) {
        PhotonLynxGetServoEnableCommand command = new PhotonLynxGetServoEnableCommand(getModule(), servo);
        try {
            command.acquireNetworkLock();
            command.send();
            return command.getResponse().thenApply(message -> {
                try {
                    command.releaseNetworkLock();
                } catch (InterruptedException e) {
                    handleException(e);
                }
                LynxGetServoEnableResponse response = (LynxGetServoEnableResponse) message;
                return response.isEnabled();
            });
        } catch (LynxNackException |InterruptedException e) {
            handleException(e);
        }
        return CompletableFuture.completedFuture(LynxUsbUtil.makePlaceholderValue(true));
    }

    @Override
    public CompletableFuture<Double> getServoPositionAsync(int servo) {
        PhotonLynxGetServoPulseWidthCommand command = new PhotonLynxGetServoPulseWidthCommand(getModule(), servo);
        try {
            command.acquireNetworkLock();
            command.send();
            return command.getResponse().thenApply(message -> {
                try {
                    command.releaseNetworkLock();
                } catch (InterruptedException e) {
                    handleException(e);
                }
                LynxGetServoPulseWidthResponse response = (LynxGetServoPulseWidthResponse) message;
                PwmControl.PwmRange range = getServoPwmRange(servo);
                return Range.scale(response.getPulseWidth(), range.usPulseLower, range.usPulseUpper, apiPositionFirst, apiPositionLast);
            });
        } catch (LynxNackException | InterruptedException e) {
            handleException(e);
        }
        return CompletableFuture.completedFuture(LynxUsbUtil.makePlaceholderValue(0.0));
    }
}

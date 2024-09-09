package com.outoftheboxrobotics.photoncore.hardware.servo;

import android.content.Context;

import com.outoftheboxrobotics.photoncore.hardware.servo.commands.PhotonLynxGetServoEnableCommand;
import com.outoftheboxrobotics.photoncore.hardware.servo.commands.PhotonLynxGetServoPulseWidthCommand;
import com.outoftheboxrobotics.photoncore.hardware.servo.commands.PhotonLynxSetServoPulseWidthCommand;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.hardware.lynx.LynxServoController;
import com.qualcomm.hardware.lynx.LynxUsbUtil;
import com.qualcomm.hardware.lynx.commands.core.LynxGetServoEnableResponse;
import com.qualcomm.hardware.lynx.commands.core.LynxGetServoPulseWidthResponse;
import com.qualcomm.hardware.lynx.commands.core.LynxSetServoEnableCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxSetServoPulseWidthCommand;
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

    @Override
    public synchronized void setServoPosition(int servo, double position) {
        double pwm = Range.scale(position, apiPositionFirst, apiPositionLast, pwmRanges[servo].usPulseLower, pwmRanges[servo].usPulseUpper);
        pwm = Range.clip(pwm, LynxSetServoPulseWidthCommand.apiPulseWidthFirst, LynxSetServoPulseWidthCommand.apiPulseWidthLast);
        PhotonLynxSetServoPulseWidthCommand command = new PhotonLynxSetServoPulseWidthCommand(this.getModule(), servo, (int)pwm);
        try {
            command.send();
        }
        catch (InterruptedException|RuntimeException|LynxNackException e)
        {
            handleException(e);
        }

        // Auto-enable after setting position to match historical behavior (and because it's handy)
        // Photon note: We don't parallelize it because it should run like. Once. Hopefully.
        internalSetPwmEnable(servo, true);
    }

    private void internalSetPwmEnable(int servoZ, boolean enable)
    {
        // Don't change state if we know we are already there
        if (lastKnownEnabled[servoZ].updateValue(enable))
        {
            // If we're disabling, then make sure that next setServoPosition will reenable
            if (!enable)
            {
                lastKnownCommandedPosition[servoZ].invalidate();
            }

            LynxSetServoEnableCommand command = new LynxSetServoEnableCommand(this.getModule(), servoZ, enable);
            try {
                command.send();
            }
            catch (InterruptedException|RuntimeException|LynxNackException e)
            {
                handleException(e);
            }
        }
    }
}

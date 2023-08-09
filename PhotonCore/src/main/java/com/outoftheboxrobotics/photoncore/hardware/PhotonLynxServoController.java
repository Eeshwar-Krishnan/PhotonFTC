package com.outoftheboxrobotics.photoncore.hardware;

import android.content.Context;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxServoController;
import com.qualcomm.hardware.lynx.LynxUnsupportedCommandException;
import com.qualcomm.hardware.lynx.commands.core.LynxSetServoPulseWidthCommand;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.util.Range;

public class PhotonLynxServoController extends LynxServoController {
    public PhotonLynxServoController(Context context, LynxModule module) throws RobotCoreException, InterruptedException {
        super(context, module);
    }

    @Override
    public void setServoPosition(int servo, double position) {
        if (lastKnownCommandedPosition[servo].updateValue(position)) {
            double pwm = Range.scale(position, apiPositionFirst, apiPositionLast, pwmRanges[servo].usPulseLower, pwmRanges[servo].usPulseUpper);
            pwm = Range.clip(pwm, LynxSetServoPulseWidthCommand.apiPulseWidthFirst, LynxSetServoPulseWidthCommand.apiPulseWidthLast);
            LynxSetServoPulseWidthCommand command = new LynxSetServoPulseWidthCommand(this.getModule(), servo, (int) pwm);
            try {
                getModule().sendCommand(command);
            } catch (InterruptedException | RuntimeException | LynxUnsupportedCommandException e) {
                handleException(e);
            }
        }
    }
}

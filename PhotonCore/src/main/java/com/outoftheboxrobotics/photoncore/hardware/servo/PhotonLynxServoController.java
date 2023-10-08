package com.outoftheboxrobotics.photoncore.hardware.servo;

import androidx.annotation.NonNull;

import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.outoftheboxrobotics.photoncore.PhotonLastKnown;
import com.outoftheboxrobotics.photoncore.hardware.PhotonLynxModule;
import com.outoftheboxrobotics.photoncore.hardware.servo.commands.PhotonLynxGetServoEnableCommand;
import com.outoftheboxrobotics.photoncore.hardware.servo.commands.PhotonLynxGetServoPulseWidthCommand;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.hardware.lynx.LynxServoController;
import com.qualcomm.hardware.lynx.LynxUnsupportedCommandException;
import com.qualcomm.hardware.lynx.LynxUsbUtil;
import com.qualcomm.hardware.lynx.commands.core.LynxGetServoEnableResponse;
import com.qualcomm.hardware.lynx.commands.core.LynxGetServoPulseWidthResponse;
import com.qualcomm.hardware.lynx.commands.core.LynxSetServoConfigurationCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxSetServoEnableCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxSetServoPulseWidthCommand;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.ServoConfigurationType;
import com.qualcomm.robotcore.util.Range;

import java.util.concurrent.CompletableFuture;

public class PhotonLynxServoController extends LynxServoController implements PhotonServoController{
    public PhotonLynxServoController(PhotonLynxModule module) throws RobotCoreException, InterruptedException {
        super(null, module);
    }

    @Override
    protected void doHook() {
        servoProperties=createPropertiesArray();
    }

    @Override
    public void initializeHardware() {
        super.initializeHardware();
        for(ServoProperties property:servoProperties)
        {
            property.lastKnownRange.setValue(PwmControl.PwmRange.defaultRange);
        }
    }

    private static class ServoProperties {
        public PhotonLastKnown<Double> lastKnownPosition = new PhotonLastKnown<>(false);
        public PhotonLastKnown<PwmControl.PwmRange> lastKnownRange = new PhotonLastKnown<>(false);
        public PhotonLastKnown<Boolean> lastKnownEnable = new PhotonLastKnown<>(false);
    }
    private static ServoProperties[] createPropertiesArray()
    {
        ServoProperties[] properties  = new ServoProperties[LynxConstants.NUMBER_OF_SERVO_CHANNELS];
        for(int i = 0; i< LynxConstants.NUMBER_OF_SERVO_CHANNELS; i++) properties[i]= new ServoProperties();
        return properties;
    }
    private ServoProperties[] servoProperties;
    @Override
    public void forgetLastKnown() {
        
        for (ServoProperties property :
            servoProperties) {
            property.lastKnownPosition.invalidate();
            property.lastKnownEnable.invalidate();
            property.lastKnownRange.invalidate();
        }
    }

    @Override
    public void pwmEnable() {
        
        for (int servoZ = 0; servoZ < LynxConstants.NUMBER_OF_SERVO_CHANNELS; servoZ++)
        {
            internalSetPwmEnable(servoZ, true);
        }
    }

    @Override
    public void pwmDisable() {
        
        for (int servoZ = 0; servoZ < LynxConstants.NUMBER_OF_SERVO_CHANNELS; servoZ++)
        {
            internalSetPwmEnable(servoZ, false);
        }
    }
    private void internalSetPwmEnable(int servoZ, boolean enable)
    {
        
        // Don't change state if we know we are already there
        if (servoProperties[servoZ].lastKnownEnable.updateValue(enable))
        {
            // If we're disabling, then make sure that next setServoPosition will reenable
            if (!enable)
            {
                servoProperties[servoZ].lastKnownPosition.invalidate();
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
    @Override
    public void setServoPwmEnable(int servo) {
        
        if(PhotonCore.photon==null) {
            super.setServoPwmEnable(servo);
            return;
        }
        internalSetPwmEnable(servo, true);
    }

    @Override
    public void setServoPwmDisable(int servo) {
        
        if(PhotonCore.photon==null) {
            super.setServoPwmEnable(servo);
            return;
        }
        internalSetPwmEnable(servo, false);
    }

    @Override
    public CompletableFuture<Boolean> isServoPwmEnabledAsync(int servo) {
        
        if(servoProperties[servo].lastKnownEnable.isValid()) {
            return CompletableFuture.completedFuture(servoProperties[servo].lastKnownEnable.getValue());
        } else {
            PhotonLynxGetServoEnableCommand command = new PhotonLynxGetServoEnableCommand(getModule(), servo);
            try {
                command.send();
                return command.getResponse().thenApply(message -> {
                    LynxGetServoEnableResponse response = (LynxGetServoEnableResponse) message;
                    boolean isEnabled =response.isEnabled();
                    servoProperties[servo].lastKnownEnable.setValue(isEnabled);
                    return isEnabled;
                });
            } catch (LynxNackException |InterruptedException e) {
                handleException(e);
            }
        }
        return CompletableFuture.completedFuture(LynxUsbUtil.makePlaceholderValue(true));
    }

    @Override
    public  boolean isServoPwmEnabled(int servo) {
        
        if(PhotonCore.photon==null) { super.isServoPwmEnabled(servo); }
        return isServoPwmEnabledAsync(servo).join();
    }

    @Override
    public void setServoType(int servo, ServoConfigurationType servoType) {
        super.setServoType(servo, servoType);
    }

    @Override
    public void setServoPosition(int servo, double position) {
        
        if( PhotonCore.photon==null) {
            super.setServoPosition(servo, position);
            return;
        }
        position=Range.clip(position, apiPositionFirst, apiPositionLast);
        if (servoProperties[servo].lastKnownPosition.updateValue(position)||PhotonCore.photon.consistentLoopTimes())
        {
            double pwm = Range.scale(position, apiPositionFirst, apiPositionLast, servoProperties[servo].lastKnownRange.getValue().usPulseLower, servoProperties[servo].lastKnownRange.getValue().usPulseUpper);
            pwm = Range.clip(pwm, LynxSetServoPulseWidthCommand.apiPulseWidthFirst, LynxSetServoPulseWidthCommand.apiPulseWidthLast);
            LynxSetServoPulseWidthCommand command = new LynxSetServoPulseWidthCommand(this.getModule(), servo, (int)pwm);
            try {
                getModule().sendCommand(command);
            }
            catch (InterruptedException | RuntimeException | LynxUnsupportedCommandException e)
            {
                handleException(e);
            }
            // Auto-enable after setting position to match historical behavior (and because it's handy)
            setServoPwmEnable(servo);
        }
    }

    @Override
    public CompletableFuture<Double> getServoPositionAsync(int servo) {
        
        if(servoProperties[servo].lastKnownPosition.isValid())
        {
            return CompletableFuture.completedFuture(servoProperties[servo].lastKnownPosition.getValue());
        }else {
            PhotonLynxGetServoPulseWidthCommand command = new PhotonLynxGetServoPulseWidthCommand(getModule(), servo);
            try {
                command.send();
                return command.getResponse().thenApply(message -> {
                    LynxGetServoPulseWidthResponse response = (LynxGetServoPulseWidthResponse) message;
                    double position = Range.scale(response.getPulseWidth(), servoProperties[servo].lastKnownRange.getValue().usPulseLower, servoProperties[servo].lastKnownRange.getValue().usPulseUpper, apiPositionFirst, apiPositionLast);
                    servoProperties[servo].lastKnownPosition.setValue(position);
                    return position;
                });
            } catch (LynxNackException | InterruptedException e) {
                handleException(e);
            }
        }
        return CompletableFuture.completedFuture(LynxUsbUtil.makePlaceholderValue(0.0));
    }

    @Override
    public  double getServoPosition(int servo) {
        
        if(PhotonCore.photon==null) return super.getServoPosition(servo);
        return getServoPositionAsync(servo).join();
    }

    @Override
    public  void setServoPwmRange(int servo, @NonNull PwmControl.PwmRange range) {
        
        if(PhotonCore.photon==null){
            super.setServoPwmRange(servo, range);
            return;
        }
        if (servoProperties[servo].lastKnownRange.updateValue(range))
        {
            LynxSetServoConfigurationCommand command = new LynxSetServoConfigurationCommand(this.getModule(), servo, (int)range.usFrame);
            try {
                command.send();
            }
            catch (InterruptedException|RuntimeException|LynxNackException e)
            {
                handleException(e);
            }
        }
    }

    @Override
    public CompletableFuture<PwmControl.PwmRange> getServoPwmRangeAsync(int servo) {
        
        return CompletableFuture.completedFuture(servoProperties[servo].lastKnownRange.getValue());
        // Apparently it's literally impossible to get the actual range from the hardware so.... pray
    }

    @NonNull
    @Override
    public  PwmControl.PwmRange getServoPwmRange(int servo) {
        
        if(PhotonCore.photon==null) return super.getServoPwmRange(servo);
        return getServoPwmRangeAsync(servo).join();
    }
}

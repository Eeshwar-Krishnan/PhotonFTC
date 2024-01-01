package com.outoftheboxrobotics.photoncore.hardware.servo;

import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.ServoConfigurationType;
import com.qualcomm.robotcore.util.Range;

import java.util.concurrent.CompletableFuture;

public class PhotonCRServo extends CRServoImplEx {
    private final PhotonLynxServoController controller;
    public PhotonCRServo(PhotonLynxServoController controller, int portNumber) {
        super(controller, portNumber, ServoConfigurationType.getStandardServoType());
        this.controller=controller;
    }
    CompletableFuture<Boolean> isPwmEnabledAsync()
    {
        return controller.isServoPwmEnabledAsync(this.portNumber);
    }
    CompletableFuture<Double> getPowerAsync()
    {
        return controller.getServoPositionAsync(this.portNumber).thenApply(position->{
            double power = Range.scale(position, apiServoPositionMin, apiServoPositionMax, apiPowerMin, apiPowerMax);
            if(direction==Direction.REVERSE)
            {
                power*=-1;
            }
            return power;
        });
    }
    CompletableFuture<Direction> getDirectionAsync()
    {
        return CompletableFuture.completedFuture(direction);
    }
}

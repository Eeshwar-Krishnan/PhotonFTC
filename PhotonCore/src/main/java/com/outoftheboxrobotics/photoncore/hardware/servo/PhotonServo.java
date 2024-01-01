package com.outoftheboxrobotics.photoncore.hardware.servo;

import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.ServoConfigurationType;

import java.util.concurrent.CompletableFuture;

public class PhotonServo extends ServoImplEx {
    private PhotonLynxServoController controller;
    public PhotonServo(PhotonLynxServoController controller, int portNumber) {
        super(controller, portNumber, ServoConfigurationType.getStandardServoType());
        this.controller=controller;
    }
    CompletableFuture<Boolean> isPwmEnabledAsync()
    {
        return controller.isServoPwmEnabledAsync(this.portNumber);
    }

    CompletableFuture<Double> getPositionAsync()
    {
        return controller.getServoPositionAsync(this.portNumber);
    }
}

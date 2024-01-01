package com.outoftheboxrobotics.photoncore.hardware.motor;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.concurrent.CompletableFuture;

/**
 * Photon class for a DC motor
 * In order to expose async functionality, it isn't enough too just override the motor controller
 */
@SuppressWarnings({"unused"})
public class PhotonDcMotor extends DcMotorImplEx {
    protected PhotonLynxDcMotorController controller;

    public PhotonDcMotor(PhotonLynxDcMotorController controller, int port){
        super(controller, port);
        this.controller=controller;
    }
    public CompletableFuture<Double> getCurrentAsync(CurrentUnit unit)
    {
        return controller.getMotorCurrentAsync(this.portNumber,unit);
    }
    public double getCorrectedCurrent(CurrentUnit unit)
    {
        return controller.getMotorCorrectedCurrent(this.portNumber,unit);
    }
    public CompletableFuture<Double> getCorrectedCurrentAsync(CurrentUnit unit)
    {
        return controller.getMotorCorrectedCurrentAsync(this.portNumber,unit);
    }
    public CompletableFuture<Boolean> isMotorEnabledAsync() {
        return controller.isMotorEnabledAsync(this.portNumber);
    }

    public CompletableFuture<PIDCoefficients> getPIDCoefficientsAsync(DcMotor.RunMode mode)
    {
        return controller.getPIDCoefficientsAsync(this.portNumber, mode);
    }
    public CompletableFuture<PIDFCoefficients> getPIDFCoefficientsAsync(DcMotor.RunMode mode)
    {
        return controller.getPIDFCoefficientsAsync(this.portNumber, mode);
    }
    public CompletableFuture<Double> getCurrentAlertAsync(CurrentUnit unit)
    {
        return controller.getMotorCurrentAlertAsync(this.portNumber, unit);
    }

    public CompletableFuture<RunMode> getMotorModeAsync()
    {
        return controller.getMotorModeAsync(this.portNumber);
    }

    public CompletableFuture<Double> getMotorPowerAsync()
    {
        return controller.getMotorPowerAsync(this.portNumber);
    }

    public CompletableFuture<ZeroPowerBehavior> getZeroPowerBehaviorAsync()
    {
        return controller.getMotorZeroPowerBehaviorAsync(this.portNumber);
    }
    public CompletableFuture<Boolean> getPowerFloatAsync()
    {
        return controller.getMotorPowerFloatAsync(this.portNumber);
    }
    public CompletableFuture<Integer> getMotorTargetPositionAsync()
    {
        return controller.getMotorTargetPositionAsync(this.portNumber);
    }

}


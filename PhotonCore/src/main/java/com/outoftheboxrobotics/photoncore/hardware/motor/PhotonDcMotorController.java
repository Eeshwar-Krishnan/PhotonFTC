package com.outoftheboxrobotics.photoncore.hardware.motor;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.concurrent.CompletableFuture;

interface PhotonDcMotorController extends DcMotorControllerEx {

     CompletableFuture<PIDCoefficients> getPIDCoefficientsAsync(int motor, DcMotor.RunMode mode);

     CompletableFuture<PIDFCoefficients> getPIDFCoefficientsAsync(int motor, DcMotor.RunMode mode);

     double getMotorCorrectedCurrent(int motor, CurrentUnit unit);

     CompletableFuture<Double> getMotorCorrectedCurrentAsync(int motor, CurrentUnit unit);
     CompletableFuture<Double> getMotorCurrentAsync(int motor, CurrentUnit unit);
     
     CompletableFuture<Double> getMotorCurrentAlertAsync(int motor, CurrentUnit unit);

     CompletableFuture<DcMotor.RunMode> getMotorModeAsync(int motor);
     
     CompletableFuture<Double> getMotorPowerAsync(int motor);
     
     CompletableFuture<DcMotor.ZeroPowerBehavior> getMotorZeroPowerBehaviorAsync(int motor);
     
     CompletableFuture<Boolean> getMotorPowerFloatAsync(int motor);
    
     CompletableFuture<Integer> getMotorTargetPositionAsync(int motor);

     CompletableFuture<Boolean> isMotorEnabledAsync(int motor);

}

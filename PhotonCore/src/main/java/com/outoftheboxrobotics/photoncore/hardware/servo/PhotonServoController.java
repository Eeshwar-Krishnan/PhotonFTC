package com.outoftheboxrobotics.photoncore.hardware.servo;


import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoControllerEx;

import java.util.concurrent.CompletableFuture;

public interface PhotonServoController extends ServoControllerEx {
    CompletableFuture<Boolean> isServoPwmEnabledAsync(int servo);

    CompletableFuture<Double> getServoPositionAsync(int servo);
}

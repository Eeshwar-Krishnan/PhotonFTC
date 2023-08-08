package com.outoftheboxrobotics.photoncore;

import com.outoftheboxrobotics.photoncore.HAL.Motors.PhotonDcMotor;
import com.outoftheboxrobotics.photoncore.HAL.PhotonHAL;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.concurrent.CompletableFuture;
import java.util.concurrent.atomic.AtomicReference;

public class Test extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        PhotonCore.start(hardwareMap);

        PhotonHAL hal = PhotonCore.getControlHubHAL();
        PhotonDcMotor motor1 = hal.getMotor(0);

        motor1.setPower(1);

        waitForStart();

        PhotonCore.experimental.setBulkDataUpdateMs(15);

        PeriodicSupplier<Double> current = new PeriodicSupplier<>(() -> motor1.getCurrentAsync(CurrentUnit.AMPS), 25);

        while (opModeIsActive()) {
            motor1.setPower(Math.sin(System.currentTimeMillis() / 1000.0));
            telemetry.addData("Current", current.get());
            telemetry.update();
        }
    }
}

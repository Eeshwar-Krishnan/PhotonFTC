package com.outoftheboxrobotics.photoncore.Examples;

import com.outoftheboxrobotics.photoncore.HAL.Motors.PhotonDcMotor;
import com.outoftheboxrobotics.photoncore.PeriodicSupplier;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

@TeleOp
@Photon(maximumParallelCommands = 10, consistentLoopTimes = true)
public class AdvancedPhotonExample extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        PhotonDcMotor motor = (PhotonDcMotor) hardwareMap.dcMotor.get("motor");
        waitForStart();

        AtomicReference<Double> current = new AtomicReference<>();
        AtomicBoolean currentInFlight = new AtomicBoolean(true);
        motor.getCurrentAsync(CurrentUnit.AMPS).thenAccept((val) -> {
            current.set(val);
            currentInFlight.set(false);
        });

        while(opModeIsActive()){
            if(!currentInFlight.get()){
                currentInFlight.set(true);
                motor.getCurrentAsync(CurrentUnit.AMPS).thenAccept((val) -> {
                    current.set(val);
                    currentInFlight.set(false);
                });
            }
            telemetry.addData("Current", current.get());
            telemetry.update();
            motor.setPower(Math.sin(System.currentTimeMillis()/1000.0));
        }
    }
}

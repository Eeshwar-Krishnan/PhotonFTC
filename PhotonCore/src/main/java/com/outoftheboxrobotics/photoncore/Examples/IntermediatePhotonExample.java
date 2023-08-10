package com.outoftheboxrobotics.photoncore.Examples;

import com.outoftheboxrobotics.photoncore.HAL.Motors.PhotonDcMotor;
import com.outoftheboxrobotics.photoncore.PeriodicSupplier;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
@TeleOp
@Photon(maximumParallelCommands = 10)
public class IntermediatePhotonExample extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        PhotonDcMotor motor = (PhotonDcMotor) hardwareMap.dcMotor.get("motor");
        waitForStart();

        PeriodicSupplier<Double> current = new PeriodicSupplier<>(() -> motor.getCurrentAsync(CurrentUnit.AMPS), 100);
        while(opModeIsActive()){
            telemetry.addData("Current", current.get());
            telemetry.update();
            motor.setPower(Math.sin(System.currentTimeMillis()/1000.0));
        }
    }
}
package com.outoftheboxrobotics.photoncore.Examples;

import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Photon
@TeleOp
public class BasicPhotonExample extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx motor = (DcMotorEx)hardwareMap.dcMotor.get("motor");
        waitForStart();
        while(opModeIsActive()){
            telemetry.addData("Current", motor.getCurrent(CurrentUnit.AMPS));
            telemetry.update();
            motor.setPower(Math.sin(System.currentTimeMillis() / 1000.0));
        }
    }
}

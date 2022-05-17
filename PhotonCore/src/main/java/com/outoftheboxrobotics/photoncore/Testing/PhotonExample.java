package com.outoftheboxrobotics.photoncore.Testing;

import com.outoftheboxrobotics.photoncore.Caching.MotorCurrentCacheIntent;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class PhotonExample extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        //PhotonCore.addCacheIntent(new BulkReadCacheIntent(5, PhotonCore.CONTROL_HUB)); //Cache bulk data every 5 ms
        PhotonCore.addCacheIntent(new MotorCurrentCacheIntent(30, PhotonCore.CONTROL_HUB, 0)); //Cache motor 0 current every 30 ms
        PhotonCore.addCacheIntent(new MotorCurrentCacheIntent(30, PhotonCore.CONTROL_HUB, 1)); //Cache motor 1 current every 30 ms

        LynxModule module = hardwareMap.getAll(LynxModule.class).get(0);
        DcMotorEx motor = (DcMotorEx) hardwareMap.dcMotor.get("motor 1");
        DcMotor motor1 = hardwareMap.dcMotor.get("motor 2");
        telemetry.setMsTransmissionInterval(50);

        double power = 0;
        long start = System.currentTimeMillis();
        double sign = 1;
        while(!isStarted()){
            long now = System.currentTimeMillis();

            double dt = (now - start) / 2000.0;

            power += dt * sign;

            motor.setPower(power);
            motor1.setPower(power);

            start = now;

            if(power > 0.2){
                sign = -1;
            }
            if(power < -0.2){
                sign = 1;
            }

            long msStart = System.currentTimeMillis();
            double current = motor.getCurrent(CurrentUnit.MILLIAMPS);
            long msEnd = System.currentTimeMillis();

            telemetry.addData("Loop Frequency", (1.0 / ((dt) / 1000.0)));
            telemetry.addData("Time Taken To Read Current", (msEnd - msStart));
            telemetry.addData("Power", current);
            telemetry.addData("Set Power", power);
            telemetry.update();
        }
        PhotonCore.enable(); //Enable PhotonCore
        power = 0;
        start = System.currentTimeMillis();
        sign = 1;
        while(opModeIsActive()){
            long now = System.currentTimeMillis();

            double dt = (now - start);

            power += (dt/2000.0) * sign;

            motor.setPower(power);
            motor1.setPower(power);

            start = now;

            if(power > 0.2){
                sign = -1;
            }
            if(power < -0.2){
                sign = 1;
            }

            long msStart = System.currentTimeMillis();
            double current = motor.getCurrent(CurrentUnit.MILLIAMPS);
            long msEnd = System.currentTimeMillis();

            telemetry.addData("Loop Frequency", (1.0 / ((dt) / 1000.0)));
            telemetry.addData("Time Taken To Read Current", (msEnd - msStart));
            telemetry.addData("Power", current);
            telemetry.addData("Set Power", power);
            telemetry.update();
        }
    }
}

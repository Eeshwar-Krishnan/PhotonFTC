package com.outoftheboxrobotics.photoncore.Testing;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.RobotLog;

import com.outoftheboxrobotics.photoncore.Caching.BulkReadCacheIntent;
import com.outoftheboxrobotics.photoncore.Caching.MotorCurrentCacheIntent;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp
public class PhotonTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        PhotonCore.addCacheIntent(new BulkReadCacheIntent(5, PhotonCore.CONTROL_HUB));
        PhotonCore.addCacheIntent(new MotorCurrentCacheIntent(30, PhotonCore.CONTROL_HUB, 0));

        LynxModule module = hardwareMap.getAll(LynxModule.class).get(0);
        SmartLynxModule smartLynxModule = new SmartLynxModule(module);
        DcMotorEx motor = smartLynxModule.getMotor(0);
        DcMotor motor1 = smartLynxModule.getMotor(1);

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

            if(Math.abs(power) > 0.2){
                sign = -sign;
            }

            long msStart = System.currentTimeMillis();
            double current = motor.getCurrent(CurrentUnit.MILLIAMPS);
            long msEnd = System.currentTimeMillis();

            telemetry.addData("dt", dt * 2000);
            telemetry.addData("Power", current);
            telemetry.update();
        }
        PhotonCore.enable();
        power = 0;
        start = System.currentTimeMillis();
        sign = 1;
        while(opModeIsActive()){
            long now = System.currentTimeMillis();

            double dt = (now - start) / 2000.0;

            power += dt * sign;

            long timer1 = System.currentTimeMillis();
            motor.setPower(power);
            long timer2 = System.currentTimeMillis();
            motor1.setPower(power);
            long timer3 = System.currentTimeMillis();

            RobotLog.i("Timing: " + (timer2 - timer1) + " motor1 | " + (timer3 - timer2) + " motor2");

            start = now;

            if(Math.abs(power) > 0.2){
                sign = -sign;
            }

            long msStart = System.currentTimeMillis();
            double current = motor.getCurrent(CurrentUnit.MILLIAMPS);
            long msEnd = System.currentTimeMillis();

            telemetry.addData("dt", dt * 2000);
            telemetry.addData("Power", current);
            telemetry.update();
        }
    }
}

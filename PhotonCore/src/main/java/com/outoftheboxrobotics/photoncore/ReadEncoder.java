package com.outoftheboxrobotics.photoncore;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp
public class ReadEncoder extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor encoder1 = hardwareMap.dcMotor.get("encoder");
        waitForStart();
        while(opModeIsActive()){
            telemetry.addData("Encoder", encoder1.getCurrentPosition());
            telemetry.update();
        }
    }
}

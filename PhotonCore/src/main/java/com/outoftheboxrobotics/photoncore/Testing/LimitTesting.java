package com.outoftheboxrobotics.photoncore.Testing;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.commands.core.LynxSetMotorConstantPowerCommand;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import com.outoftheboxrobotics.photoncore.PhotonCore;

@TeleOp
public class LimitTesting extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        LynxModule module = hardwareMap.getAll(LynxModule.class).get(0);
        waitForStart();
        for(int i = 0; i < 100; i ++){
            LynxSetMotorConstantPowerCommand command = new LynxSetMotorConstantPowerCommand(module, 0, (int) Range.scale(Math.random(), -1, 1, -LynxSetMotorConstantPowerCommand.apiPowerFirst, LynxSetMotorConstantPowerCommand.apiPowerLast));
            PhotonCore.testSend(command);
        }
        Thread.sleep(50);
        LynxSetMotorConstantPowerCommand command = new LynxSetMotorConstantPowerCommand(module, 0, 0);
        PhotonCore.testSend(command);
        while (opModeIsActive()){
            telemetry.addLine("Done");
            telemetry.update();
        }
    }
}

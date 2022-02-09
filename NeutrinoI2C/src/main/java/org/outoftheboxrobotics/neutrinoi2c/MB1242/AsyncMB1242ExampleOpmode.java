package org.outoftheboxrobotics.neutrinoi2c.MB1242;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class AsyncMB1242ExampleOpmode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        AsyncMB1242 sensor = hardwareMap.get(AsyncMB1242.class, "mb1242");

        waitForStart();
        while(opModeIsActive()){
            telemetry.addData("Distance", sensor.getDistance(DistanceUnit.CM));

            telemetry.update();
        }
    }
}

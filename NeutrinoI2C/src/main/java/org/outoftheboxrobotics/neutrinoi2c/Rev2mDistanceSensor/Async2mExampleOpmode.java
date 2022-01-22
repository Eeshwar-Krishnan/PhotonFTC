package org.outoftheboxrobotics.neutrinoi2c.Rev2mDistanceSensor;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class Async2mExampleOpmode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Rev2mDistanceSensor sensor = hardwareMap.get(Rev2mDistanceSensor.class, "distancesensor");
        AsyncRev2MSensor asyncSensor = new AsyncRev2MSensor(sensor);

        telemetry.addLine("Press a for high accuracy, x for balanced accuracy, and b for high speed");

        waitForStart();
        while(opModeIsActive()){
            telemetry.addData("Distance", asyncSensor.getDistance(DistanceUnit.CM));
            telemetry.addData("Last Reading", asyncSensor.getLastMeasurementTimestamp());

            if(gamepad1.a){
                asyncSensor.setSensorAccuracyMode(AsyncRev2MSensor.AccuracyMode.MODE_HIGH_ACCURACY);
            }
            if(gamepad1.x){
                asyncSensor.setSensorAccuracyMode(AsyncRev2MSensor.AccuracyMode.MODE_BALANCED);
            }
            if(gamepad1.b){
                asyncSensor.setSensorAccuracyMode(AsyncRev2MSensor.AccuracyMode.MODE_HIGH_SPEED);
            }

            telemetry.update();
        }
    }
}

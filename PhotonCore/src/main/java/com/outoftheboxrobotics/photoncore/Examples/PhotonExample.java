package com.outoftheboxrobotics.photoncore.Examples;

import com.outoftheboxrobotics.photoncore.HAL.Motors.PhotonDcMotor;
import com.outoftheboxrobotics.photoncore.HAL.PhotonHAL;
import com.outoftheboxrobotics.photoncore.PeriodicSupplier;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.concurrent.atomic.AtomicReference;

public class PhotonExample extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        /**
         * Start PhotonCore
         * Required, recommended to be near first in the opmode
         */
        PhotonCore.start(hardwareMap);

        /**
         * HAL is your main friend when it comes to using PhotonFTC.
         * You will need two instances of the HAL, one for each hub
         *
         * The built in control hub HAL is always an instance of PhotonHAL
         * Expansion hubs (coming soon) will either be PhotonHAL if connected over usb,
         * or RS485HAL if connected over RS485
         * (RS485HAL will contain no special features due to photon incompatibility with rs485)
         */
        PhotonHAL hal = PhotonCore.getControlHubHAL();

        /**
         * Here we get a motor using the HAL. Indexing is done by port.
         * Coming soon: Auto replacement of hardwaremap motor instances with PhotonDcMotor
         *
         * The default return of the hal is a DcMotorEx instance
         * since hal is an instance of PhotonHAL we can cast that to PhotonDcMotor
         */
        PhotonDcMotor motor1 = (PhotonDcMotor) hal.getMotor(0);

        /**
         * DcMotorEx functions are the same in PhotonDcMotor
         */
        motor1.setPower(1);

        waitForStart();

        /**
         * This does absolutely nothing right now
         */
        PhotonCore.experimental.setBulkDataUpdateMs(15);

        /**
         * Here we show an example of PeriodicSupplier, to get motor current asynchronously
         * Period is defined in milliseconds, and the function in the lambda will be run
         * This means that every 25 milliseconds, the value in the supplier will be updated
         *
         * BE CAREFUL when using this. If you overload your HAL (too many requests per second)
         * then you will experience issues of overcrowding and delays
         * If a value isn't important, let it update slowly.
         */
        PeriodicSupplier<Double> current = new PeriodicSupplier<>(() -> motor1.getCurrentAsync(CurrentUnit.AMPS), 25);

        boolean a_prev = false;

        /**
         * Set up a second value to track current to show off async references
         * Value must be atomic since its referenced from inside a callback
         * which is called in a different thread
         */
        AtomicReference<Double> currentAsync2 = new AtomicReference<>(0.0);
        while (opModeIsActive()) {
            motor1.setPower(Math.sin(System.currentTimeMillis() / 1000.0));
            telemetry.addData("Current", current.get());
            if(gamepad1.a && !a_prev){
                /**
                 * getCurrentAsync returns a CompletableFuture which will return the current value
                 * here we set up a callback when the value is returned to update the value in currentAsync2 with it
                 */
                motor1.getCurrentAsync(CurrentUnit.AMPS).thenAccept(currentAsync2::set);
                /**
                 * As a note, the above is a method reference. Its basically shorthand for
                 * motor1.getCurrentAsync(CurrentUnit.AMPS).thenAccept(val -> currentAsync2.set(val));
                 */
            }
            a_prev = gamepad1.a;

            telemetry.addData("Current 2", currentAsync2.get());

            telemetry.update();
        }
    }
}

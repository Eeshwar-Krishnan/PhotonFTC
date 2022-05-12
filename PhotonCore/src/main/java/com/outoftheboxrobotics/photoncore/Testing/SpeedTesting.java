package com.outoftheboxrobotics.photoncore.Testing;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.commands.core.LynxSetMotorConstantPowerCommand;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import com.outoftheboxrobotics.photoncore.Commands.V2.LynxGetBulkInputDataCommand;
import com.outoftheboxrobotics.photoncore.PhotonCore;

import java.util.ArrayList;
import java.util.Locale;
import java.util.concurrent.atomic.AtomicBoolean;

@TeleOp
public class SpeedTesting extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        LynxModule module = hardwareMap.getAll(LynxModule.class).get(0);
        SmartLynxModule smartLynxModule = new SmartLynxModule(module);
        DcMotor motor = smartLynxModule.getMotor(0);
        DcMotor motor1 = smartLynxModule.getMotor(1);
        DcMotor motor2 = smartLynxModule.getMotor(2);

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double power = 0, sign = 1;
        long start = System.currentTimeMillis();

        AtomicBoolean runMotor = new AtomicBoolean(true);

        Thread thread = new Thread(() -> {
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            RobotLog.ii("Pos", "Started");
            long past = System.nanoTime();
            double sum = 0, len = 0, encoderVal = 0, timer = 0;
            ArrayList<LynxGetBulkInputDataCommand> list = new ArrayList<>();
            ArrayList<Long> timeStamps = new ArrayList<>();
            while (opModeIsActive()){
                long nsStart = System.nanoTime();
                LynxGetBulkInputDataCommand command = new LynxGetBulkInputDataCommand(module);
                PhotonCore.testSend(command);
                long now = System.nanoTime();
                double dt = (now - past) / 1e+9;
                sum += (1.0/dt);
                len++;
                past = now;

                list.add(command);
                timeStamps.add(System.currentTimeMillis());

                if(list.get(0).isResponded()){
                    encoderVal = (double)list.get(0).getResponse().getEncoder(0);
                    list.remove(0);
                    timer = (System.currentTimeMillis() - timeStamps.get(0));
                    timeStamps.remove(0);
                }

                while(list.size() > 4){
                    if(list.get(0).isResponded()){
                        encoderVal = (double)list.get(0).getResponse().getEncoder(0);
                        list.remove(0);
                        timer = (System.currentTimeMillis() - timeStamps.get(0));
                        timeStamps.remove(0);
                    }
                }

                if(opModeIsActive()) {
                    RobotLog.ii("Pos", String.format(Locale.US, "%2f hz, %2f value, %2f buffer, %2f timer", sum/len, encoderVal, (double)list.size(), timer));

                    double diff = (25000 - encoderVal)/100;
                    if(Math.abs(diff) > 1){
                        diff = Math.signum(diff);
                    }

                    double power1 = Range.scale(diff, -1, 1, LynxSetMotorConstantPowerCommand.apiPowerFirst, LynxSetMotorConstantPowerCommand.apiPowerLast);
                    LynxSetMotorConstantPowerCommand command1 = new LynxSetMotorConstantPowerCommand(module, 1, (Math.abs(encoderVal - 50000) < 30) ? 0 : (int) power1);
                    //PhotonCore.testSend(command1);
                }
                long end = System.nanoTime();
                while(end - nsStart < (1e+6)){
                    end = System.nanoTime();
                }
            }
        });
        thread.start();

        while(opModeIsActive()){
            long now = System.currentTimeMillis();

            double dt = (now - start) / 2000.0;

            power += dt * sign;

            motor.setPower(power);
            motor1.setPower(power);

            start = now;

            if(Math.abs(power) > 1){
                sign = -sign;
            }

            telemetry.addData("dt", dt * 1000);
            telemetry.addData("Power", power);
            telemetry.update();
        }
    }
}

package com.outoftheboxrobotics.photoncore.Commands;

import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataResponse;

import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;

public class BulkData
        {
        private final LynxGetBulkInputDataResponse resp;
        private final boolean fake;

        public BulkData(LynxGetBulkInputDataResponse resp, boolean fake)
            {
            this.resp = resp;
            this.fake = fake;
            }

        public boolean getDigitalChannelState(int digitalInputZ)
            {
            return resp.getDigitalInput(digitalInputZ);
            }

        public int getMotorCurrentPosition(int motorZ)
            {
            return resp.getEncoder(motorZ);
            }

        /** Returns (signed) motor velocity in encoder counts per second */
        public int getMotorVelocity(int motorZ)
            {
            return resp.getVelocity(motorZ);
            }

        public boolean isMotorBusy(int motorZ)
            {
            return !resp.isAtTarget(motorZ);
            }

        public boolean isMotorOverCurrent(int motorZ)
            {
            return resp.isOverCurrent(motorZ);
            }

        /** Returns the analog input in V */
        public double getAnalogInputVoltage(int inputZ)
            {
            return getAnalogInputVoltage(inputZ, VoltageUnit.VOLTS);
            }

        public double getAnalogInputVoltage(int inputZ, VoltageUnit unit)
            {
            return unit.convert(resp.getAnalogInput(inputZ), VoltageUnit.MILLIVOLTS);
            }

        public boolean isFake()
            {
            return fake;
            }
        }
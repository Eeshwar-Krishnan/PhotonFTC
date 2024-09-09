package com.outoftheboxrobotics.photoncore.hardware.motor;

import com.qualcomm.robotcore.hardware.DcMotor;

public class PhotonAdvancedDcMotor {
    private final PhotonDcMotor motor;

    private double cacheTolerance = 0.001;
    private double powerMemory = 0;

    private boolean setRefreshRate = false;

    public PhotonAdvancedDcMotor(PhotonDcMotor motor) {
        this.motor = motor;
    }

    /**
     * Sets the tolerance under which a write will not execute for a motor.
     */
    public void setCacheTolerance(double cacheTolerance) {
        this.cacheTolerance = cacheTolerance;
    }

    /**
     * Gets the current cache tolerance under which a write will not execute for a motor.
     */
    public double getCacheTolerance() {
        return cacheTolerance;
    }

    /**
     * ADVANCED USE ONLY. USE WITH CAUTION
     *
     * This allows Photon to automatically set a "set refresh rate", which prevents writes from occurring faster then the specified threshold.
     * Set power of 0 will automatically pass through instantly, otherwise it will be held until the next window. Setting the rate to 0ms will disable this system.
     *
     * IMPORTANT: This method is NOT threaded internally. This means that setPower() or update() must be REGULARLY CALLED for this to work.
     * Timing is evaluated internally on a setPower() or update() call, so the speed of that call is the precision with which this method will work.
     *
     * CRITICAL: This method does NOT work with RUN_TO_POSITION.
     */
    public void setMotorSetRefreshRate(long rateMs) {
        motor.controller.setMotorRefreshRate(motor.port, rateMs);
        setRefreshRate = !(rateMs == 0);
    }

    /**
     * ADVANCED USE ONLY. USE WITH CAUTION
     *
     * This is part of a system that allows Photon to automatically set a "set refresh rate", which prevents writes from occurring faster then the specified threshold.
     *
     * IMPORTANT: This method is NOT threaded internally. This means that setPower() or update() must be REGULARLY CALLED for this to work.
     * Timing is evaluated internally on a setPower() or update() call, so the speed of that call is the precision with which this method will work.
    */
    public void update() {
        motor.controller.updateRefreshes(motor.port);
    }


    /**
     * Sets the motor power
     * @return if the motor power was set, or if it was within the cache tolerance
     */

    public boolean setPower(double power){
        return internalSetPower(power);
    }

    private boolean internalSetPower(double power) {
        if(Math.abs(power - powerMemory) > cacheTolerance) {
            powerMemory = power;
            if(setRefreshRate) {
                motor.controller.setMotorPowerAdv(motor.port, power);
            }else {
                motor.setPower(power);
            }
            return true;
        }
        if(setRefreshRate) {
            motor.controller.updateRefreshes(motor.port);
        }
        return false;
    }

    /**
     * Gets the underlying motor, to do non-optimzed write related tasks with
     */
    public PhotonDcMotor getMotor() {
        return motor;
    }
}

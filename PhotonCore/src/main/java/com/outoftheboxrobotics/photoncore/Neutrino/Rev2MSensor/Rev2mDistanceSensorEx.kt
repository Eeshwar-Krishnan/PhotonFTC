package com.outoftheboxrobotics.photoncore.Neutrino.Rev2MSensor

import com.qualcomm.hardware.lynx.LynxI2cDeviceSynch
import com.qualcomm.hardware.rev.Rev2mDistanceSensor
import com.qualcomm.robotcore.hardware.I2cDeviceSynch
import com.qualcomm.robotcore.util.ElapsedTime
import com.qualcomm.robotcore.util.TypeConversion

class Rev2mDistanceSensorEx(deviceClient: I2cDeviceSynch?, owned: Boolean) : Rev2mDistanceSensor(deviceClient, owned) {
    var measurement_timing_budget_us: Long = 0
    protected var io_timeout = 0
    protected var ioElapsedTime: ElapsedTime? = null
    private var lastRead: Long = 0
    private var rangeCache = 0

    init {
        if (deviceClient is LynxI2cDeviceSynch) {
            (deviceClient as LynxI2cDeviceSynch).setBusSpeed(LynxI2cDeviceSynch.BusSpeed.FAST_400K)
        }
    }

    override fun readRangeContinuousMillimeters(): Int {
        if (io_timeout > 0) {
            ioElapsedTime.reset()
        }
        // assumptions: Linearity Corrective Gain is 1000 (default);
        // fractional ranging is not enabled
        if (System.currentTimeMillis() - lastRead > measurement_timing_budget_us / 1000.0) {
            val readBytes = deviceClient.read(Register.RESULT_RANGE_STATUS.bVal + 10, 2)
            rangeCache = TypeConversion.byteArrayToShort(readBytes).toInt()
            lastRead = System.currentTimeMillis()
        }
        return rangeCache
    }

    /**
     * Sets ranging profile of the device
     * DEFAULT: 30 hz update, medium accuracy
     * HIGH_SPEED: 50 hz update, low accuracy
     * HIGH_ACCURACY: 10 hz update, high accuracy
     */
    fun setRangingProfile(rangingProfile: RANGING_PROFILE?) {
        when (rangingProfile) {
            RANGING_PROFILE.DEFAULT -> setMeasurementTimingBudget(33000)
            RANGING_PROFILE.HIGH_SPEED -> setMeasurementTimingBudget(20000)
            RANGING_PROFILE.HIGH_ACCURACY -> setMeasurementTimingBudget(100000)
            else -> {}
        }
    }

    enum class RANGING_PROFILE {
        DEFAULT,
        HIGH_SPEED,
        HIGH_ACCURACY
    }
}

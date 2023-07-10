/*
Copyright (c) 2019 REV Robotics LLC

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of REV Robotics LLC nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package com.outoftheboxrobotics.photoncore.Neutrino.RevColorSensor

import com.qualcomm.hardware.broadcom.BroadcomColorSensor
import com.qualcomm.hardware.broadcom.BroadcomColorSensor.*
import com.qualcomm.hardware.lynx.LynxI2cDeviceSynch
import com.qualcomm.robotcore.hardware.*
import com.qualcomm.robotcore.hardware.HardwareDevice.Manufacturer
import com.qualcomm.robotcore.util.Range
import com.qualcomm.robotcore.util.RobotLog
import com.qualcomm.robotcore.util.TypeConversion
import java.nio.ByteOrder

/**
 * BroadcomColorSensorImpl is used to support the Rev Robotics V3 color sensor
 */
abstract class BroadcomColorSensorImplEx protected constructor(
    params: BroadcomColorSensor.Parameters?,
    deviceClient: I2cDeviceSynchSimple?,
    isOwned: Boolean
) : I2cDeviceSynchDeviceWithParameters<I2cDeviceSynchSimple?, BroadcomColorSensor.Parameters>(
    deviceClient,
    isOwned,
    params!!
), BroadcomColorSensor, I2cAddrConfig, Light {
    var colors = NormalizedRGBA()
    var red = 0
    var green = 0
    var blue = 0
    var alpha = 0
    var softwareGain = 1f
    var lastRead: Long = 0
    var measurementDelay: Long = 100

    //----------------------------------------------------------------------------------------------
    // Construction
    //----------------------------------------------------------------------------------------------
    init {
        this.deviceClient!!.logging = parameters.loggingEnabled
        this.deviceClient!!.loggingTag = parameters.loggingTag

        // We ask for an initial call back here; that will eventually call internalInitialize()
        registerArmingStateCallback(true)
        engage()
        internalInitialize(parameters)
        if (deviceClient is LynxI2cDeviceSynch) {
            deviceClient.setBusSpeed(LynxI2cDeviceSynch.BusSpeed.FAST_400K)
        }
    }

    //----------------------------------------------------------------------------------------------
    // Initialization
    //----------------------------------------------------------------------------------------------
    @Synchronized
    override fun internalInitialize(parameters: BroadcomColorSensor.Parameters): Boolean {
        RobotLog.vv(TAG, "internalInitialize()...")
        return try {
            require(this.parameters.deviceId == parameters.deviceId) {
                String.format(
                    "can't change device types (modify existing params instead): old=%d new=%d",
                    this.parameters.deviceId,
                    parameters.deviceId
                )
            }

            // Remember the parameters for future use
            this.parameters = parameters.clone()

            // Make sure we're talking to the correct I2c device
            this.i2cAddress = parameters.i2cAddr

            // Can't do anything if we're not really talking to the hardware
            if (!deviceClient!!.isArmed) {
                return false
            }

            // Verify that that's a color sensor!
            val id = this.deviceID
            if (id.toInt() != parameters.deviceId) {
                RobotLog.ee(
                    TAG,
                    "unexpected Broadcom color sensor chipid: found=%d expected=%d",
                    id,
                    parameters.deviceId
                )
                return false
            }

            // sanity check: before
            dumpState()

            // set the gain, LED parameters and resolution
            setHardwareGain(parameters.gain)
            setLEDParameters(parameters.pulseModulation, parameters.ledCurrent)
            setProximityPulseCount(parameters.proximityPulseCount)
            setPSRateAndRes(
                BroadcomColorSensor.Parameters.proximityResolution,
                parameters.proximityMeasRate
            )
            setLSRateAndRes(
                BroadcomColorSensor.Parameters.lightSensorResolution,
                parameters.lightSensorMeasRate
            )

            // Enable the device
            enable()

            // sanity check: after
            dumpState()

            // Set up a read-ahead, if supported and requested
            if (deviceClient is I2cDeviceSynch && parameters.readWindow != null) {
                val synch = deviceClient as I2cDeviceSynch?
                synch!!.readWindow = parameters.readWindow
            }
            true
        } finally {
            RobotLog.vv(TAG, "...internalInitialize()")
        }
    }

    protected fun dumpState() {
        val cb = 0x07
        RobotLog.logBytes(TAG, "state", read(BroadcomColorSensor.Register.MAIN_CTRL, cb), cb)
    }

    @Synchronized
    protected fun enable() {
        var enabled = readMainCtrl()
        RobotLog.vv(TAG, "enable() enabled=0x%02x...", enabled)

        // enable ambient light sensor, ALS/IR channels, and proximity
        write8(
            BroadcomColorSensor.Register.MAIN_CTRL,
            MainControl.PS_EN.bVal.toInt() or MainControl.LS_EN.bVal.toInt() or MainControl.RGB_MODE.bVal.toInt()
        )
        enabled = readMainCtrl()
        RobotLog.vv(TAG, "...enable() enabled=0x%02x", enabled)
        return
    }

    @Synchronized
    protected fun disable() {
        var enabled = readMainCtrl()
        RobotLog.vv(TAG, "disable() enabled=0x%02x...", enabled)
        write8(
            BroadcomColorSensor.Register.MAIN_CTRL,
            enabled.toInt() and (MainControl.PS_EN.bVal.toInt() or MainControl.LS_EN.bVal.toInt() or MainControl.RGB_MODE.bVal.toInt()).inv()
        )
        enabled = readMainCtrl()
        RobotLog.vv(TAG, "...disable() enabled=0x%02x", enabled)
    }

    protected fun testBits(value: Byte, desired: Byte): Boolean {
        return testBits(value, desired, desired)
    }

    protected fun testBits(value: Byte, mask: Byte, desired: Byte): Boolean {
        return value.toInt() and mask.toInt() == desired.toInt()
    }

    protected fun readMainCtrl(): Byte {
        return read8(BroadcomColorSensor.Register.MAIN_CTRL)
    }

    protected fun setProximityPulseCount(proximityPulseCount: Int) {
        RobotLog.vv(TAG, "setProximityPulseCount(0x%02x)", proximityPulseCount)
        write8(BroadcomColorSensor.Register.PS_PULSES, proximityPulseCount)
    }

    protected fun setHardwareGain(gain: BroadcomColorSensor.Gain) {
        RobotLog.vv(TAG, "setGain(0x%02x)", gain.bVal)
        write8(BroadcomColorSensor.Register.LS_GAIN, gain.bVal.toInt())
    }

    protected fun setPDrive(ledDrive: LEDCurrent) {
        RobotLog.vv(TAG, "setPDrive(0x%02x)", ledDrive.bVal)
        write8(BroadcomColorSensor.Register.PS_LED, ledDrive.bVal.toInt())
    }

    override fun getDeviceID(): Byte {
        return read8(BroadcomColorSensor.Register.PART_ID)
    }

    protected fun setLEDParameters(pulseMod: LEDPulseModulation, curr: LEDCurrent) {
        val `val` = pulseMod.bVal.toInt() shl 4 or curr.bVal.toInt()
        RobotLog.vv(TAG, "setLEDParameters(0x%02x)", `val`.toByte())
        write8(BroadcomColorSensor.Register.PS_LED, `val`.toByte().toInt())
    }

    protected fun setPSRateAndRes(res: PSResolution, rate: PSMeasurementRate) {
        val `val` = res.bVal.toInt() shl 3 or rate.bVal.toInt()
        RobotLog.vv(TAG, "setPSMeasRate(0x%02x)", `val`.toByte())
        write8(BroadcomColorSensor.Register.PS_MEAS_RATE, `val`.toByte().toInt())
        when (rate) {
            PSMeasurementRate.RES -> {}
            PSMeasurementRate.R6_25ms -> measurementDelay = 7
            PSMeasurementRate.R12_5ms -> measurementDelay = 13
            PSMeasurementRate.R25ms -> measurementDelay = 25
            PSMeasurementRate.R50ms -> measurementDelay = 50
            PSMeasurementRate.R100ms -> measurementDelay = 100
            PSMeasurementRate.R200ms -> measurementDelay = 200
            PSMeasurementRate.R400ms -> measurementDelay = 400
        }
    }

    protected fun setLSRateAndRes(res: LSResolution, rate: LSMeasurementRate) {
        val `val` = res.bVal.toInt() shl 4 or rate.bVal.toInt()
        RobotLog.vv(TAG, "setLSMeasRate(0x%02x)", `val`.toByte())
        write8(BroadcomColorSensor.Register.LS_MEAS_RATE, `val`.toByte().toInt())
    }
    //----------------------------------------------------------------------------------------------
    // Interfaces
    //----------------------------------------------------------------------------------------------
    /** In this implementation, the [Color] methods return 16 bit unsigned values.  */
    @Synchronized
    override fun red(): Int {
        updateColors()
        return red
    }

    @Synchronized
    override fun green(): Int {
        updateColors()
        return green
    }

    @Synchronized
    override fun blue(): Int {
        updateColors()
        return blue
    }

    @Synchronized
    override fun alpha(): Int {
        updateColors()
        return alpha
    }

    @Synchronized
    override fun argb(): Int {
        return normalizedColors.toColor()
    }

    override fun setGain(newGain: Float) {
        softwareGain = newGain
    }

    override fun getGain(): Float {
        return softwareGain
    }

    private fun updateColors() {
        val now = System.currentTimeMillis()
        if (now - lastRead > measurementDelay) {
            val data: ByteArray

            // Read red, green and blue values
            val cbRead = 9
            data = read(BroadcomColorSensor.Register.LS_DATA_GREEN, cbRead)
            val dib = 0
            green =
                TypeConversion.unsignedShortToInt(TypeConversion.byteArrayToShort(data, dib, ByteOrder.LITTLE_ENDIAN))
            blue = Range.clip(
                (1.55 * TypeConversion.unsignedShortToInt(
                    TypeConversion.byteArrayToShort(
                        data, dib + 3, ByteOrder.LITTLE_ENDIAN
                    )
                )).toInt(), 0, 65535
            )
            red = Range.clip(
                (1.07 * TypeConversion.unsignedShortToInt(
                    TypeConversion.byteArrayToShort(
                        data, dib + 6, ByteOrder.LITTLE_ENDIAN
                    )
                )).toInt(), 0, 65535
            )
            alpha = (red + green + blue) / 3

            // normalize to [0, 1]
            colors.red = Range.clip(red.toFloat() * softwareGain / parameters.colorSaturation, 0f, 1f)
            colors.green = Range.clip(green.toFloat() * softwareGain / parameters.colorSaturation, 0f, 1f)
            colors.blue = Range.clip(blue.toFloat() * softwareGain / parameters.colorSaturation, 0f, 1f)

            // apply inverse squared law of light to get readable brightness value, stored in alpha channel
            // scale to 65535
            val avg = (red + green + blue).toFloat() / 3
            colors.alpha = (-(65535f / (Math.pow(avg.toDouble(), 2.0) + 65535)) + 1).toFloat()
            lastRead = System.currentTimeMillis()
        }
    }

    override fun getNormalizedColors(): NormalizedRGBA {
        updateColors()
        return colors
    }

    @Synchronized
    override fun enableLed(enable: Boolean) {
        // We can't directly control the LED with I2C; it's always on
        // ignore; used to throw an error, but the default opmode can try to turn off
        // (for range sensor variant) so got constant exceptions
    }

    override fun isLightOn(): Boolean {
        return true
    }

    @Synchronized
    override fun getI2cAddress(): I2cAddr {
        return deviceClient!!.i2cAddress
    }

    @Synchronized
    override fun setI2cAddress(i2cAddr: I2cAddr) {
        parameters.i2cAddr = i2cAddr
        deviceClient!!.i2cAddress = i2cAddr
    }

    //----------------------------------------------------------------------------------------------
    // HardwareDevice
    //----------------------------------------------------------------------------------------------
    override fun getDeviceName(): String {
        return "Broadcom I2C Color Sensor"
    }

    override fun getManufacturer(): Manufacturer {
        return Manufacturer.Broadcom
    }

    override fun resetDeviceConfigurationForOpMode() {
        super.resetDeviceConfigurationForOpMode()
        softwareGain = 1f
    }

    //----------------------------------------------------------------------------------------------
    // Utility
    //----------------------------------------------------------------------------------------------
    protected fun readUnsignedByte(reg: BroadcomColorSensor.Register): Int {
        return TypeConversion.unsignedByteToInt(read8(reg))
    }

    protected fun readUnsignedShort(reg: BroadcomColorSensor.Register, byteOrder: ByteOrder?): Int {
        val data = read(reg, 2)
        return TypeConversion.unsignedShortToInt(TypeConversion.byteArrayToShort(data, 0, byteOrder))
    }

    @Synchronized
    override fun read8(reg: BroadcomColorSensor.Register): Byte {
        return deviceClient!!.read8(reg.bVal.toInt())
    }

    @Synchronized
    override fun read(reg: BroadcomColorSensor.Register, cb: Int): ByteArray {
        return deviceClient!!.read(reg.bVal.toInt(), cb)
    }

    @Synchronized
    override fun write8(reg: BroadcomColorSensor.Register, data: Int) {
        deviceClient!!.write8(reg.bVal.toInt(), data, I2cWaitControl.WRITTEN)
    }

    override fun write(reg: BroadcomColorSensor.Register, data: ByteArray) {
        deviceClient!!.write(reg.bVal.toInt(), data, I2cWaitControl.WRITTEN)
    }

    protected fun delay(ms: Int) {
        try {
            Thread.sleep(ms.toLong())
        } catch (e: InterruptedException) {
            Thread.currentThread().interrupt()
        }
    }

    companion object {
        //----------------------------------------------------------------------------------------------
        // State
        //----------------------------------------------------------------------------------------------
        const val TAG = "BroadcomColorSensorImpl"
    }
}

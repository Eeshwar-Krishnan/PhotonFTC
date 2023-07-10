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
import com.qualcomm.hardware.broadcom.BroadcomColorSensor.PSMeasurementRate
import com.qualcomm.hardware.broadcom.BroadcomColorSensor.PSResolution
import com.qualcomm.hardware.rev.RevColorSensorV3
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple
import com.qualcomm.robotcore.hardware.NormalizedRGBA
import com.qualcomm.robotcore.util.Range
import com.qualcomm.robotcore.util.RobotLog
import com.qualcomm.robotcore.util.TypeConversion
import java.nio.ByteOrder

/**
 * [RevColorSensorV3Ex] implements support for the REV Robotics Color Sensor V3.
 *
 * @see [REV Robotics Website](http://revrobotics.com)
 */
class RevColorSensorV3Ex(deviceClient: I2cDeviceSynchSimple?, owned: Boolean) : RevColorSensorV3(deviceClient, owned) {
    var colors = NormalizedRGBA()
    var lastRead: Long = 0
    var measurementDelay: Long = 100
    var red = 0
    var green = 0
    var blue = 0
    var alpha = 0
    override fun setPSRateAndRes(res: PSResolution, rate: PSMeasurementRate) {
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

    @Synchronized
    override fun red(): Int {
        updateColors()
        return this.red
    }

    @Synchronized
    override fun green(): Int {
        updateColors()
        return this.green
    }

    @Synchronized
    override fun blue(): Int {
        updateColors()
        return this.blue
    }

    @Synchronized
    override fun alpha(): Int {
        updateColors()
        return this.alpha
    }

    @Synchronized
    override fun argb(): Int {
        return normalizedColors.toColor()
    }

    private fun updateColors() {
        val now = System.currentTimeMillis()
        if (now - lastRead > measurementDelay) {
            val data: ByteArray

            // Read red, green and blue values
            val cbRead = 9
            data = read(BroadcomColorSensor.Register.LS_DATA_GREEN, cbRead)
            val dib = 0
            this.green =
                TypeConversion.unsignedShortToInt(TypeConversion.byteArrayToShort(data, dib, ByteOrder.LITTLE_ENDIAN))
            this.blue = Range.clip(
                (1.55 * TypeConversion.unsignedShortToInt(
                    TypeConversion.byteArrayToShort(
                        data, dib + 3, ByteOrder.LITTLE_ENDIAN
                    )
                )).toInt(), 0, 65535
            )
            this.red = Range.clip(
                (1.07 * TypeConversion.unsignedShortToInt(
                    TypeConversion.byteArrayToShort(
                        data, dib + 6, ByteOrder.LITTLE_ENDIAN
                    )
                )).toInt(), 0, 65535
            )
            this.alpha = (this.red + this.green + this.blue) / 3

            // normalize to [0, 1]
            this.colors.red = Range.clip(this.red.toFloat() * gain / parameters.colorSaturation, 0f, 1f)
            this.colors.green = Range.clip(this.green.toFloat() * gain / parameters.colorSaturation, 0f, 1f)
            this.colors.blue = Range.clip(this.blue.toFloat() * gain / parameters.colorSaturation, 0f, 1f)

            // apply inverse squared law of light to get readable brightness value, stored in alpha channel
            // scale to 65535
            val avg = (this.red + this.green + this.blue).toFloat() / 3
            this.colors.alpha = (-(65535f / (Math.pow(avg.toDouble(), 2.0) + 65535)) + 1).toFloat()
            lastRead = System.currentTimeMillis()
        }
    }

    override fun getNormalizedColors(): NormalizedRGBA {
        updateColors()
        return this.colors
    }

    fun setUpdateRate(updateRate: UPDATE_RATE?) {
        when (updateRate) {
            UPDATE_RATE.DEFAULT -> setPSRateAndRes(PSResolution.R11BIT, PSMeasurementRate.R100ms)
            UPDATE_RATE.HIGH_SPEED -> setPSRateAndRes(PSResolution.R11BIT, PSMeasurementRate.R6_25ms)
            else -> {}
        }
    }

    enum class UPDATE_RATE {
        DEFAULT,
        HIGH_SPEED
    }
}

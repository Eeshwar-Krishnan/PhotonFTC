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
package com.outoftheboxrobotics.photoncore.Neutrino.RevColorSensor;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.TypeConversion;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.nio.ByteOrder;
import java.util.Locale;

/**
 * {@link RevColorSensorV3Ex} implements support for the REV Robotics Color Sensor V3.
 *
 * @see <a href="http://revrobotics.com">REV Robotics Website</a>
 *
 */
@SuppressWarnings("WeakerAccess")
public class RevColorSensorV3Ex extends RevColorSensorV3
{
    NormalizedRGBA colors = new NormalizedRGBA();
    long lastRead = 0, measurementDelay = 100;
    int red = 0, green = 0, blue = 0, alpha = 0;

    public RevColorSensorV3Ex(I2cDeviceSynchSimple deviceClient, boolean owned)
    {
        super(deviceClient, owned);
    }

    @Override
    protected void setPSRateAndRes(PSResolution res, PSMeasurementRate rate)
    {
        int val = (res.bVal << 3) | rate.bVal;
        RobotLog.vv(TAG, "setPSMeasRate(0x%02x)", (byte) val);
        write8(Register.PS_MEAS_RATE, (byte) val);

        switch (rate){
            case RES:
                break;
            case R6_25ms:
                measurementDelay = 7;
                break;
            case R12_5ms:
                measurementDelay = 13;
                break;
            case R25ms:
                measurementDelay = 25;
                break;
            case R50ms:
                measurementDelay = 50;
                break;
            case R100ms:
                measurementDelay = 100;
                break;
            case R200ms:
                measurementDelay = 200;
                break;
            case R400ms:
                measurementDelay = 400;
                break;
        }
    }

    @Override
    public synchronized int red() { updateColors(); return this.red; }

    @Override
    public synchronized int green() { updateColors(); return this.green; }

    @Override
    public synchronized int blue() { updateColors(); return this.blue; }

    @Override
    public synchronized int alpha() { updateColors(); return this.alpha; }

    @Override
    public synchronized int argb() { return getNormalizedColors().toColor(); }

    private void updateColors()
    {
        long now = System.currentTimeMillis();
        if(now - lastRead > measurementDelay) {
            byte[] data;

            // Read red, green and blue values
            final int cbRead = 9;
            data = read(Register.LS_DATA_GREEN, cbRead);

            final int dib = 0;
            this.green = TypeConversion.unsignedShortToInt(TypeConversion.byteArrayToShort(data, dib, ByteOrder.LITTLE_ENDIAN));
            this.blue = Range.clip((int) (1.55 * TypeConversion.unsignedShortToInt(TypeConversion.byteArrayToShort(
                    data, dib + 3, ByteOrder.LITTLE_ENDIAN))), 0, 65535);
            this.red = Range.clip((int) (1.07 * TypeConversion.unsignedShortToInt(TypeConversion.byteArrayToShort(
                    data, dib + 6, ByteOrder.LITTLE_ENDIAN))), 0, 65535);

            this.alpha = (this.red + this.green + this.blue) / 3;

            // normalize to [0, 1]
            this.colors.red = Range.clip(((float) this.red * getGain()) / parameters.colorSaturation, 0f, 1f);
            this.colors.green = Range.clip(((float) this.green * getGain()) / parameters.colorSaturation, 0f, 1f);
            this.colors.blue = Range.clip(((float) this.blue * getGain()) / parameters.colorSaturation, 0f, 1f);

            // apply inverse squared law of light to get readable brightness value, stored in alpha channel
            // scale to 65535
            float avg = (float) (this.red + this.green + this.blue) / 3;
            this.colors.alpha = (float) (-(65535f / (Math.pow(avg, 2) + 65535)) + 1);

            lastRead = System.currentTimeMillis();
        }
    }

    @Override
    public NormalizedRGBA getNormalizedColors() { updateColors(); return this.colors; }

    public void setUpdateRate(UPDATE_RATE updateRate){
        switch (updateRate){
            case DEFAULT:
                setPSRateAndRes(PSResolution.R11BIT, PSMeasurementRate.R100ms);
                break;
            case HIGH_SPEED:
                setPSRateAndRes(PSResolution.R11BIT, PSMeasurementRate.R6_25ms);
                break;
        }
    }

    public enum UPDATE_RATE{
        DEFAULT,
        HIGH_SPEED
    }
}

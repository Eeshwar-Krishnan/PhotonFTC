package com.outoftheboxrobotics.photoncore.Neutrino.MB1242

import com.outoftheboxrobotics.photoncore.ReflectionUtils.getField
import com.qualcomm.hardware.lynx.LynxI2cDeviceSynch
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.hardware.lynx.LynxNackException
import com.qualcomm.hardware.lynx.commands.LynxCommand
import com.qualcomm.hardware.lynx.commands.core.LynxI2cReadMultipleBytesCommand
import com.qualcomm.hardware.lynx.commands.core.LynxI2cReadStatusQueryCommand
import com.qualcomm.hardware.lynx.commands.core.LynxI2cWriteReadMultipleBytesCommand
import com.qualcomm.hardware.lynx.commands.standard.LynxNack.StandardReasonCode
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerImpl
import com.qualcomm.robotcore.exception.RobotCoreException
import com.qualcomm.robotcore.hardware.*
import com.qualcomm.robotcore.hardware.HardwareDevice.Manufacturer
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType
import com.qualcomm.robotcore.util.TypeConversion
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit

@I2cDeviceType
@DeviceProperties(name = "Photon MB1242", description = "ultrasonic distance sensor", xmlTag = "MB1242")
class MB1242Ex(i2cDeviceSynch: I2cDeviceSynch) : I2cDeviceSynchDevice<I2cDeviceSynch?>(i2cDeviceSynch, true),
    DistanceSensor {
    private var module: LynxModule? = null
    private var bus = 0
    private var address: I2cAddr? = null
    private var lastRun: Long = 0
    private var lastReading: Short = 0

    init {
        deviceClient!!.i2cAddress = I2cAddr.create7bit(0x70)
        registerArmingStateCallback(false)
        deviceClient!!.engage()
        setupVariables(i2cDeviceSynch)
    }

    fun getDistanceAsync(unit: DistanceUnit): Double {
        if (System.currentTimeMillis() > lastRun + 20) {
            try {
                module!!.acquireI2cLockWhile<Any?> {
                    val tx: LynxCommand<*> = LynxI2cWriteReadMultipleBytesCommand(module, bus, address, 0, 2)
                    tx.send()
                    val data = pollForReadResult(2)
                    if (data.size == 2) {
                        lastReading = TypeConversion.byteArrayToShort(data)
                        deviceClient!!.write(TypeConversion.intToByteArray(0x51))
                        lastRun = System.currentTimeMillis()
                    }
                    null
                }
            } catch (e: InterruptedException) {
                e.printStackTrace()
            } catch (e: RobotCoreException) {
                e.printStackTrace()
            } catch (e: LynxNackException) {
                e.printStackTrace()
            }
        }
        return unit.fromCm(lastReading.toDouble())
    }

    override fun getDistance(unit: DistanceUnit): Double {
        try {
            module!!.acquireI2cLockWhile<Any?> {
                var tx: LynxCommand<*> = LynxI2cReadMultipleBytesCommand(module, bus, address, 2)
                tx.send()
                var data = pollForReadResult(2)
                while (data.size == 0) {
                    tx = LynxI2cReadMultipleBytesCommand(module, bus, address, 2)
                    try {
                        tx.send()
                    } catch (e: InterruptedException) {
                        e.printStackTrace()
                    } catch (e: LynxNackException) {
                        e.printStackTrace()
                    }
                    data = pollForReadResult(2)
                }
                if (data.size == 2) {
                    lastReading = TypeConversion.byteArrayToShort(data)
                    deviceClient!!.write(TypeConversion.intToByteArray(0x51))
                    lastRun = System.currentTimeMillis()
                }
                null
            }
        } catch (e: InterruptedException) {
            e.printStackTrace()
        } catch (e: RobotCoreException) {
            e.printStackTrace()
        } catch (e: LynxNackException) {
            e.printStackTrace()
        }
        return unit.fromCm(lastReading.toDouble())
    }

    override fun getManufacturer(): Manufacturer {
        return Manufacturer.Other
    }

    override fun getDeviceName(): String {
        return "MB1242 sensor"
    }

    override fun getConnectionInfo(): String {
        return "Connected lol"
    }

    override fun getVersion(): Int {
        return 1
    }

    override fun doInitialize(): Boolean {
        return true
    }

    override fun resetDeviceConfigurationForOpMode() {}
    override fun close() {}
    private fun setupVariables(i2cDeviceSynch: I2cDeviceSynch) {
        var device: LynxI2cDeviceSynch? = null
        try {
            val simple = i2cDeviceSynch as I2cDeviceSynchImplOnSimple
            //Cool first part done
            //Now we can safely make the assumption that the underlying i2cdevicesynchsimple is a lynxi2cdevicesynch
            val field = getField(simple.javaClass, "i2cDeviceSynchSimple")
            field!!.isAccessible = true
            device = field[simple] as LynxI2cDeviceSynch
            //Lets also bump up the bus speed while we are here
            //Tbh it doesn't affect anything when just reading 2 bytes but why not
            device!!.setBusSpeed(LynxI2cDeviceSynch.BusSpeed.FAST_400K)
        } catch (e: IllegalAccessException) {
            e.printStackTrace()
        }
        var module: LynxModule? = null
        try {
            //Module this is being run on, sometimes the module doesn't exist
            module = getField(device!!.javaClass, "module")!![device] as LynxModule
        } catch (e: IllegalAccessException) {
            e.printStackTrace()
        }
        this.module = module
        var bus = 0
        try {
            //Bus that the i2c device is on. Why is this one the only one IntelliJ is mad at?
            bus = getField(device!!.javaClass, "bus")!![device] as Int
        } catch (e: IllegalAccessException) {
            e.printStackTrace()
        }
        this.bus = bus
        address = I2cAddr.create7bit(0x70)
    }

    protected fun pollForReadResult(creg: Int): ByteArray {
        // Poll until the data is available
        var keepTrying = true
        while (keepTrying) {
            val readStatus = LynxI2cReadStatusQueryCommand(module, bus, creg)
            keepTrying = try {
                val response = readStatus.sendReceive()
                val now = System.nanoTime()
                return response.bytes
            } catch (e: LynxNackException) {
                when (e.nack.nackReasonCodeAsEnum) {
                    StandardReasonCode.I2C_MASTER_BUSY, StandardReasonCode.I2C_OPERATION_IN_PROGRESS ->                         // We used to sleep for 3ms while waiting for the result to avoid a "busy loop", but that
                        // caused a serious performance hit over what we could get otherwise, at least on the CH.
                        // Besides, we're not *truly* busy looping, we still end up waiting for the module's response
                        // and what not.

                        //try { Thread.sleep(msBusyWait); } catch (InterruptedException ignored) { Thread.currentThread().interrupt(); }
                        continue  // This is an internal error of some sort
                    else -> false
                }
            } catch (e: InterruptedException) {
                false
            } catch (e: RuntimeException) {
                false
            }
        }
        return ByteArray(0)
    }

    companion object {
        private val opModeManager: OpModeManagerImpl? = null
    }
}
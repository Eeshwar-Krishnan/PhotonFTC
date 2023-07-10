package com.outoftheboxrobotics.photoncore.HAL.I2C

import android.content.Context
import com.outoftheboxrobotics.photoncore.HAL.HAL
import com.outoftheboxrobotics.photoncore.HAL.I2C.Commands.PhotonLynxI2cReadMultipleBytesCommand
import com.outoftheboxrobotics.photoncore.HAL.I2C.Commands.PhotonLynxI2cReadSingleByteCommand
import com.outoftheboxrobotics.photoncore.HAL.I2C.Commands.PhotonLynxI2cWriteSingleByteCommand
import com.qualcomm.hardware.lynx.LynxI2cDeviceSynchV1
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.hardware.lynx.LynxNackException
import com.qualcomm.hardware.lynx.Supplier
import com.qualcomm.hardware.lynx.commands.LynxCommand
import com.qualcomm.hardware.lynx.commands.core.LynxI2cWriteSingleByteCommand
import com.qualcomm.hardware.lynx.commands.standard.LynxNack.StandardReasonCode
import com.qualcomm.robotcore.exception.RobotCoreException
import com.qualcomm.robotcore.hardware.I2cWaitControl
import com.qualcomm.robotcore.hardware.I2cWarningManager
import com.qualcomm.robotcore.hardware.TimestampedData
import com.qualcomm.robotcore.hardware.TimestampedI2cData

class PhotonLynxI2cDeviceSyncV1(private val hal: HAL, context: Context?, module: LynxModule?, bus: Int) :
    LynxI2cDeviceSynchV1(context, module, bus), PhotonLynxI2cDeviceSynch {
    private var interleavedCommand: LynxCommand<*>? = null
    override fun setInterleavedCommand(command: LynxCommand<*>?) {
        interleavedCommand = command
    }

    override fun clearInterleavedCommand() {
        interleavedCommand = null
    }

    @Synchronized
    override fun readTimeStamped(ireg: Int, creg: Int): TimestampedData {
        try {
            val writeTxSupplier = Supplier<LynxI2cWriteSingleByteCommand> {
                PhotonLynxI2cWriteSingleByteCommand(module, bus, i2cAddr, ireg)
            }
            val readTxSupplier = Supplier<LynxCommand<*>> {
                if (creg == 1) PhotonLynxI2cReadSingleByteCommand(module, bus, i2cAddr) else PhotonLynxI2cReadMultipleBytesCommand(module, bus, i2cAddr, creg)
            }
            return acquireI2cLockWhile {
                sendI2cTransaction(writeTxSupplier)
                internalWaitForWriteCompletions(I2cWaitControl.ATOMIC)
                sendI2cTransaction(readTxSupplier)
                if (interleavedCommand != null) hal.write(interleavedCommand)
                readTimeStampedPlaceholder.reset()
                pollForReadResult(i2cAddr, ireg, creg)
            }
        } catch (e: InterruptedException) {
            handleException(e)
        } catch (e: RobotCoreException) {
            handleException(e)
        } catch (e: RuntimeException) {
            handleException(e)
        } catch (e: LynxNackException) {
            /*
             * This is a possible device problem, go ahead and tell I2cWarningManager to warn.
             */
            I2cWarningManager.notifyProblemI2cDevice(this)
            handleException(e)
        }
        return readTimeStampedPlaceholder.log(TimestampedI2cData.makeFakeData(i2cAddress, ireg, creg))
    }

    @Throws(LynxNackException::class, InterruptedException::class, RobotCoreException::class)
    override fun sendI2cTransaction(transactionSupplier: Supplier<out LynxCommand<*>?>) {
        var attempts = 0;
        while (attempts<5) {
            try {
                hal.write(transactionSupplier.get())
                break
            } catch (e: LynxNackException) {
                attempts++
                when (e.nack.nackReasonCodeAsEnum) {
                    StandardReasonCode.I2C_MASTER_BUSY, StandardReasonCode.I2C_OPERATION_IN_PROGRESS -> {}
                    else -> throw e
                }
                if (attempts == 5) {
                    error("I2C transaction failed after 5 attempts, giving up.")
                }
            }
        }
    } // TODO: override internalWaitForWriteCompletions and pollForResults when hal has sendReceive
}
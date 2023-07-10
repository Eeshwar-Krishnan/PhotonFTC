package com.outoftheboxrobotics.photoncore.HAL.Motors

import com.outoftheboxrobotics.photoncore.HAL.HAL
import com.outoftheboxrobotics.photoncore.HAL.Motors.Commands.PhotonLynxGetMotorChannelEnableCommand
import com.outoftheboxrobotics.photoncore.HAL.Motors.Commands.PhotonLynxGetMotorPIDControlLoopCoefficientsCommand
import com.outoftheboxrobotics.photoncore.HAL.Motors.Commands.PhotonLynxSetMotorChannelEnableCommand
import com.outoftheboxrobotics.photoncore.HAL.Motors.Commands.PhotonLynxSetMotorPIDFControlLoopCoefficientsCommand
import com.outoftheboxrobotics.photoncore.HAL.Motors.Commands.PhotonLynxSetMotorTargetVelocityCommand
import com.qualcomm.hardware.lynx.LynxNackException
import com.qualcomm.hardware.lynx.commands.LynxMessage
import com.qualcomm.hardware.lynx.commands.core.*
import com.qualcomm.robotcore.hardware.*
import com.qualcomm.robotcore.hardware.DcMotor.RunMode
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior
import com.qualcomm.robotcore.hardware.HardwareDevice.Manufacturer
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType
import com.qualcomm.robotcore.util.LastKnown
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit
import java.util.concurrent.CompletableFuture
import java.util.concurrent.ExecutionException
import java.util.concurrent.TimeUnit
import java.util.concurrent.TimeoutException

open class PhotonDcMotor protected constructor(private val hal: HAL, private val port: Int) : DcMotorEx {
    private val enabled: LastKnown<Boolean>
    private val setVelocity: LastKnown<Int>
    private val velocity: LastKnown<Double>
    private val pidCoefficients: LastKnown<PIDCoefficients>
    private val pidfCoefficients: LastKnown<PIDFCoefficients>
    protected var direction = DcMotorSimple.Direction.FORWARD

    init {
        enabled = LastKnown()
        enabled.invalidate()
        setVelocity = LastKnown()
        setVelocity.invalidate()
        velocity = LastKnown()
        velocity.invalidate()
        pidCoefficients = LastKnown()
        pidCoefficients.invalidate()
        pidfCoefficients = LastKnown()
        pidfCoefficients.invalidate()
    }

    private fun adjustAngularRate(angularRate: Double): Double {
        var angularRate = angularRate
        if (direction == DcMotorSimple.Direction.REVERSE) angularRate = -angularRate
        return angularRate
    }

    override fun setMotorEnable() {
        if (enabled.updateValue(true)) {
            val command = PhotonLynxSetMotorChannelEnableCommand(
                hal.lynxModule,
                port, true
            )
            hal.write(command)
        }
    }

    override fun setMotorDisable() {
        if (enabled.updateValue(false)) {
            val command = PhotonLynxSetMotorChannelEnableCommand(
                hal.lynxModule,
                port, false
            )
            hal.write(command)
        }
    }

    override fun isMotorEnabled(): Boolean {
        if (enabled.isValid) {
            return enabled.value
        } else {
            val command: PhotonLynxGetMotorChannelEnableCommand<LynxGetMotorChannelEnableResponse> =
                PhotonLynxGetMotorChannelEnableCommand(
                    hal.lynxModule,
                    port
                )
            try {
                val message: LynxMessage = command.getResponse().get(1, TimeUnit.SECONDS)
                enabled.updateValue((message as LynxGetMotorChannelEnableResponse).isEnabled)
                return message.isEnabled
            } catch (e: ExecutionException) {
                e.printStackTrace()
            } catch (e: InterruptedException) {
                e.printStackTrace()
            } catch (e: TimeoutException) {
                e.printStackTrace()
            } catch (e: LynxNackException) {
                e.printStackTrace()
            }
            hal.write(command)
        }
        return enabled.nonTimedValue
    }

    val isMotorEnabledAsync: CompletableFuture<Boolean?>
        get() {
            var future = CompletableFuture<Boolean?>()
            if (enabled.isValid) {
                future = CompletableFuture()
                future.complete(enabled.value)
            } else {
                val command: PhotonLynxGetMotorChannelEnableCommand<LynxGetMotorChannelEnableResponse> =
                    PhotonLynxGetMotorChannelEnableCommand(
                        hal.lynxModule,
                        port
                    )
                try {
                    future = command.getResponse()
                        .thenApply { response -> (response as LynxGetMotorChannelEnableResponse).isEnabled }
                } catch (e: LynxNackException) {
                    e.printStackTrace()
                }
                hal.write(command)
            }
            return future
        }

    override fun setVelocity(angularRate: Double) {
        var angularRate = angularRate
        angularRate = adjustAngularRate(angularRate)
        when (mode) {
            RunMode.RUN_USING_ENCODER, RunMode.RUN_TO_POSITION -> {}
            else -> mode = RunMode.RUN_USING_ENCODER
        }
        val iTicksPerSecond = Range.clip(
            Math.round(angularRate).toInt(),
            LynxSetMotorTargetVelocityCommand.apiVelocityFirst,
            LynxSetMotorTargetVelocityCommand.apiVelocityLast
        )
        try {
            if (setVelocity.updateValue(iTicksPerSecond)) {
                val command = PhotonLynxSetMotorTargetVelocityCommand(
                    hal.lynxModule,
                    port, iTicksPerSecond
                )
                hal.write(command)
                setMotorEnable()
            }
        } catch (e: RuntimeException) {
            e.printStackTrace()
        }
    }

    /**
     * WARNING: THIS METHOD IS NOT IMPLEMENTED CORRECTLY
     *
     * THIS METHOD WILL ASSUME A GOBILDA 5202 MOTOR
     * USE WITH CAUTION
     */
    override fun setVelocity(angularRate: Double, unit: AngleUnit) {
        val degreesPerSecond = UnnormalizedAngleUnit.DEGREES.fromUnit(unit.unnormalized, angularRate)
        val revolutionsPerSecond = degreesPerSecond / 360.0
        val ticksPerSecond = 28 * revolutionsPerSecond
        setVelocity(ticksPerSecond)
    }

    override fun getVelocity(): Double {
        return if (velocity.isValid) {
            velocity.value
        } else {
            val vel = hal.bulkData!!.getVelocity(port).toDouble()
            velocity.updateValue(vel)
            vel
        }
    }

    override fun getVelocity(unit: AngleUnit): Double {
        val ticksPerSecond = getVelocity()
        val revsPerSecond = ticksPerSecond / 28.0
        return unit.unnormalized.fromDegrees(revsPerSecond * 360.0)
    }

    override fun setPIDCoefficients(mode: RunMode, pidCoefficients: PIDCoefficients) {
        setPIDFCoefficients(mode, PIDFCoefficients(pidCoefficients))
    }

    @Throws(UnsupportedOperationException::class)
    override fun setPIDFCoefficients(mode: RunMode, pidfCoefficients: PIDFCoefficients) {
        if (!mode.isPIDMode) {
            throw RuntimeException("RunMode " + mode.name + " is not a PID mode")
        }
        val command = PhotonLynxSetMotorPIDFControlLoopCoefficientsCommand(
            hal.lynxModule,
            port,
            mode,
            LynxSetMotorPIDControlLoopCoefficientsCommand.internalCoefficientFromExternal(pidfCoefficients.p),
            LynxSetMotorPIDControlLoopCoefficientsCommand.internalCoefficientFromExternal(pidfCoefficients.i),
            LynxSetMotorPIDControlLoopCoefficientsCommand.internalCoefficientFromExternal(pidfCoefficients.d),
            LynxSetMotorPIDControlLoopCoefficientsCommand.internalCoefficientFromExternal(pidfCoefficients.f),
            LynxSetMotorPIDFControlLoopCoefficientsCommand.InternalMotorControlAlgorithm.fromExternal(pidfCoefficients.algorithm)
        )
        hal.write(command)
    }

    override fun setVelocityPIDFCoefficients(p: Double, i: Double, d: Double, f: Double) {
        setPIDFCoefficients(RunMode.RUN_USING_ENCODER, PIDFCoefficients(p, i, d, f, MotorControlAlgorithm.PIDF))
    }

    override fun setPositionPIDFCoefficients(p: Double) {
        setPIDFCoefficients(RunMode.RUN_TO_POSITION, PIDFCoefficients(p, 0.0, 0.0, 0.0, MotorControlAlgorithm.PIDF))
    }

    override fun getPIDCoefficients(mode: RunMode): PIDCoefficients {
        if (pidCoefficients.isValid) {
            return pidCoefficients.value
        } else {
            val command = PhotonLynxGetMotorPIDControlLoopCoefficientsCommand(
                hal.lynxModule,
                port, mode
            )
            hal.write(command)
            try {
                val message: LynxMessage = command.getResponse().get(1, TimeUnit.SECONDS)
                val response = message as LynxGetMotorPIDControlLoopCoefficientsResponse
                pidCoefficients.invalidate()
                val coefficients = PIDCoefficients(
                    LynxSetMotorPIDControlLoopCoefficientsCommand.externalCoefficientFromInternal(response.p),
                    LynxSetMotorPIDControlLoopCoefficientsCommand.externalCoefficientFromInternal(response.i),
                    LynxSetMotorPIDControlLoopCoefficientsCommand.externalCoefficientFromInternal(response.d)
                )
                pidCoefficients.updateValue(coefficients)
                return coefficients
            } catch (e: ExecutionException) {
                e.printStackTrace()
            } catch (e: InterruptedException) {
                e.printStackTrace()
            } catch (e: TimeoutException) {
                e.printStackTrace()
            } catch (e: LynxNackException) {
                e.printStackTrace()
            }
        }
        return pidCoefficients.rawValue
    }

    override fun getPIDFCoefficients(mode: RunMode): PIDFCoefficients? {
        return null
    }

    override fun setTargetPositionTolerance(tolerance: Int) {}
    override fun getTargetPositionTolerance(): Int {
        return 0
    }

    override fun getCurrent(unit: CurrentUnit): Double {
        return 0.0
    }

    override fun getCurrentAlert(unit: CurrentUnit): Double {
        return 0.0
    }

    override fun setCurrentAlert(current: Double, unit: CurrentUnit) {}
    override fun isOverCurrent(): Boolean {
        return false
    }

    override fun getMotorType(): MotorConfigurationType? {
        return null
    }

    override fun setMotorType(motorType: MotorConfigurationType) {}
    override fun getController(): DcMotorController? {
        return null
    }

    override fun getPortNumber(): Int {
        return 0
    }

    override fun setZeroPowerBehavior(zeroPowerBehavior: ZeroPowerBehavior) {}
    override fun getZeroPowerBehavior(): ZeroPowerBehavior? {
        return null
    }

    @Deprecated("Deprecated in Java")
    override fun setPowerFloat() {}
    override fun getPowerFloat(): Boolean {
        return false
    }

    override fun setTargetPosition(position: Int) {}
    override fun getTargetPosition(): Int {
        return 0
    }

    override fun isBusy(): Boolean {
        return false
    }

    override fun getCurrentPosition(): Int {
        return 0
    }

    override fun setMode(mode: RunMode) {}
    override fun getMode(): RunMode? {
        return null
    }

    override fun setDirection(direction: DcMotorSimple.Direction) {}
    override fun getDirection(): DcMotorSimple.Direction? {
        return null
    }

    override fun setPower(power: Double) {}
    override fun getPower(): Double {
        return 0.0
    }

    override fun getManufacturer(): Manufacturer? {
        return null
    }

    override fun getDeviceName(): String? {
        return null
    }

    override fun getConnectionInfo(): String? {
        return null
    }

    override fun getVersion(): Int {
        return 0
    }

    override fun resetDeviceConfigurationForOpMode() {}
    override fun close() {}
}
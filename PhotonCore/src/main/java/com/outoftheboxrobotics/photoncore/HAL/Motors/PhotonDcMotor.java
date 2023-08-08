package com.outoftheboxrobotics.photoncore.HAL.Motors;

import com.outoftheboxrobotics.photoncore.HAL.HAL;
import com.outoftheboxrobotics.photoncore.HAL.Motors.Commands.PhotonLynxGetADCCommand;
import com.outoftheboxrobotics.photoncore.HAL.Motors.Commands.PhotonLynxGetMotorChannelCurrentAlertLevelCommand;
import com.outoftheboxrobotics.photoncore.HAL.Motors.Commands.PhotonLynxGetMotorChannelEnableCommand;
import com.outoftheboxrobotics.photoncore.HAL.Motors.Commands.PhotonLynxGetMotorChannelModeCommand;
import com.outoftheboxrobotics.photoncore.HAL.Motors.Commands.PhotonLynxGetMotorPIDControlLoopCoefficientsCommand;
import com.outoftheboxrobotics.photoncore.HAL.Motors.Commands.PhotonLynxGetMtorPIDFControlLoopCoefficientsCommand;
import com.outoftheboxrobotics.photoncore.HAL.Motors.Commands.PhotonLynxSetMotorChannelCurrentAlertLevelCommand;
import com.outoftheboxrobotics.photoncore.HAL.Motors.Commands.PhotonLynxSetMotorChannelEnableCommand;
import com.outoftheboxrobotics.photoncore.HAL.Motors.Commands.PhotonLynxSetMotorChannelModeCommand;
import com.outoftheboxrobotics.photoncore.HAL.Motors.Commands.PhotonLynxSetMotorConstantPowerCommand;
import com.outoftheboxrobotics.photoncore.HAL.Motors.Commands.PhotonLynxSetMotorPIDFControlLoopCoefficientsCommand;
import com.outoftheboxrobotics.photoncore.HAL.Motors.Commands.PhotonLynxSetMotorTargetPositionCommand;
import com.outoftheboxrobotics.photoncore.HAL.Motors.Commands.PhotonLynxSetMotorTargetVelocityCommand;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.hardware.lynx.commands.LynxCommand;
import com.qualcomm.hardware.lynx.commands.LynxMessage;
import com.qualcomm.hardware.lynx.commands.core.LynxGetADCCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetADCResponse;
import com.qualcomm.hardware.lynx.commands.core.LynxGetMotorChannelCurrentAlertLevelCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetMotorChannelCurrentAlertLevelResponse;
import com.qualcomm.hardware.lynx.commands.core.LynxGetMotorChannelEnableResponse;
import com.qualcomm.hardware.lynx.commands.core.LynxGetMotorChannelModeCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetMotorChannelModeResponse;
import com.qualcomm.hardware.lynx.commands.core.LynxGetMotorPIDControlLoopCoefficientsCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetMotorPIDControlLoopCoefficientsResponse;
import com.qualcomm.hardware.lynx.commands.core.LynxGetMotorPIDFControlLoopCoefficientsResponse;
import com.qualcomm.hardware.lynx.commands.core.LynxSetMotorChannelCurrentAlertLevelCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxSetMotorConstantPowerCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxSetMotorPIDControlLoopCoefficientsCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxSetMotorPIDFControlLoopCoefficientsCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxSetMotorTargetPositionCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxSetMotorTargetVelocityCommand;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.LastKnown;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Rotation;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

public class PhotonDcMotor implements DcMotorEx {
    private final HAL hal;
    private final int port;

    private final LastKnown<Boolean> enabled;
    private final LastKnown<Integer> setVelocity;
    private final LastKnown<Double> velocity;
    private final LastKnown<PIDCoefficients> pidCoefficients;
    private final LastKnown<PIDFCoefficients> pidfCoefficients;
    private final LastKnown<Double> currentLimit;
    private final LastKnown<ZeroPowerBehavior> zeroPowerBehavior;

    protected Direction direction = Direction.FORWARD;

    protected int positionTolerance = 5;

    protected PhotonDcMotor(HAL hal, int port){
        this.hal = hal;
        this.port = port;

        enabled = new LastKnown<>();
        enabled.invalidate();

        setVelocity = new LastKnown<>();
        setVelocity.invalidate();

        velocity = new LastKnown<>();
        velocity.invalidate();

        pidCoefficients = new LastKnown<>();
        pidCoefficients.invalidate();

        pidfCoefficients = new LastKnown<>();
        pidfCoefficients.invalidate();

        currentLimit = new LastKnown<>();
        currentLimit.invalidate();

        zeroPowerBehavior = new LastKnown<>();
        zeroPowerBehavior.invalidate();
    }

    protected double adjustAngularRate(double angularRate) {
        if (direction == Direction.REVERSE) angularRate = -angularRate;
        return angularRate;
    }

    @Override
    public void setMotorEnable() {
        if(enabled.updateValue(true)) {
            PhotonLynxSetMotorChannelEnableCommand command = new PhotonLynxSetMotorChannelEnableCommand(hal.getLynxModule(), port, true);
            hal.write(command);
        }
    }

    @Override
    public void setMotorDisable() {
        if(enabled.updateValue(false)) {
            PhotonLynxSetMotorChannelEnableCommand command = new PhotonLynxSetMotorChannelEnableCommand(hal.getLynxModule(), port, false);
            hal.write(command);
        }
    }

    @Override
    public boolean isMotorEnabled() {
        if(enabled.isValid()){
            return enabled.getValue();
        }else{
            PhotonLynxGetMotorChannelEnableCommand<LynxGetMotorChannelEnableResponse> command = new PhotonLynxGetMotorChannelEnableCommand<>(hal.getLynxModule(), port);
            try {
                LynxMessage message = command.getResponse().get(1, TimeUnit.SECONDS);
                enabled.updateValue(((LynxGetMotorChannelEnableResponse)message).isEnabled());
                return ((LynxGetMotorChannelEnableResponse)message).isEnabled();
            } catch (ExecutionException | InterruptedException | TimeoutException | LynxNackException e) {
                e.printStackTrace();
            }
            hal.write(command);
        }
        return enabled.getNonTimedValue();
    }

    public CompletableFuture<Boolean> isMotorEnabledAsync() {
        CompletableFuture<Boolean> future = new CompletableFuture<>();

        if(enabled.isValid()){
            future = new CompletableFuture<>();
            future.complete(enabled.getValue());
        }else{
            PhotonLynxGetMotorChannelEnableCommand<LynxGetMotorChannelEnableResponse> command = new PhotonLynxGetMotorChannelEnableCommand<>(hal.getLynxModule(), port);
            try {
                future = command.getResponse().thenApply(response -> ((LynxGetMotorChannelEnableResponse)response).isEnabled());
            } catch (LynxNackException e) {
                e.printStackTrace();
            }
            hal.write(command);
        }
        return future;
    }

    @Override
    public void setVelocity(double angularRate) {
        angularRate = adjustAngularRate(angularRate);

        switch (getMode())
        {
            case RUN_USING_ENCODER:
            case RUN_TO_POSITION:
                break;
            default:
                setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        int iTicksPerSecond = Range.clip((int)Math.round(angularRate),
                LynxSetMotorTargetVelocityCommand.apiVelocityFirst,
                LynxSetMotorTargetVelocityCommand.apiVelocityLast);

        try {
            if(setVelocity.updateValue(iTicksPerSecond)) {
                PhotonLynxSetMotorTargetVelocityCommand command = new PhotonLynxSetMotorTargetVelocityCommand(hal.getLynxModule(), port, iTicksPerSecond);
                hal.write(command);
                setMotorEnable();
            }
        }
        catch (RuntimeException e)
        {
            e.printStackTrace();
        }
    }

    /**
     * WARNING: THIS METHOD IS NOT IMPLEMENTED CORRECTLY
     *
     * THIS METHOD WILL ASSUME A GOBILDA 5202 MOTOR
     * USE WITH CAUTION
     */
    @Deprecated
    @Override
    public void setVelocity(double angularRate, AngleUnit unit) {
        double degreesPerSecond     = UnnormalizedAngleUnit.DEGREES.fromUnit(unit.getUnnormalized(), angularRate);
        double revolutionsPerSecond = degreesPerSecond / 360.0;
        double ticksPerSecond       = 28 * revolutionsPerSecond;

        setVelocity(ticksPerSecond);
    }

    @Override
    public double getVelocity() {
        if(velocity.isValid()){
            return velocity.getValue();
        }else{
            double vel = hal.getBulkData().getVelocity(port);
            velocity.updateValue(vel);
            return vel;
        }
    }

    @Override
    public double getVelocity(AngleUnit unit) {
        double ticksPerSecond = getVelocity();
        double revsPerSecond = ticksPerSecond / 28.0;
        return unit.getUnnormalized().fromDegrees(revsPerSecond * 360.0);
    }

    @Override
    public void setPIDCoefficients(RunMode mode, PIDCoefficients pidCoefficients) {
        setPIDFCoefficients(mode, new PIDFCoefficients(pidCoefficients));
    }

    @Override
    public void setPIDFCoefficients(RunMode mode, PIDFCoefficients pidfCoefficients) throws UnsupportedOperationException {
        if(!mode.isPIDMode()){
            throw new RuntimeException("RunMode " + mode.name() + " is not a PID mode");
        }
        PhotonLynxSetMotorPIDFControlLoopCoefficientsCommand command = new PhotonLynxSetMotorPIDFControlLoopCoefficientsCommand(
                hal.getLynxModule(),
                port,
                mode,
                LynxSetMotorPIDControlLoopCoefficientsCommand.internalCoefficientFromExternal(pidfCoefficients.p),
                LynxSetMotorPIDControlLoopCoefficientsCommand.internalCoefficientFromExternal(pidfCoefficients.i),
                LynxSetMotorPIDControlLoopCoefficientsCommand.internalCoefficientFromExternal(pidfCoefficients.d),
                LynxSetMotorPIDControlLoopCoefficientsCommand.internalCoefficientFromExternal(pidfCoefficients.f),
                LynxSetMotorPIDFControlLoopCoefficientsCommand.InternalMotorControlAlgorithm.fromExternal(pidfCoefficients.algorithm));

        hal.write(command);
    }

    @Override
    public void setVelocityPIDFCoefficients(double p, double i, double d, double f) {
        setPIDFCoefficients(RunMode.RUN_USING_ENCODER, new PIDFCoefficients(p, i, d, f, MotorControlAlgorithm.PIDF));
    }

    @Override
    public void setPositionPIDFCoefficients(double p) {
        setPIDFCoefficients(RunMode.RUN_TO_POSITION, new PIDFCoefficients(p, 0, 0, 0, MotorControlAlgorithm.PIDF));
    }

    @Override
    public PIDCoefficients getPIDCoefficients(RunMode mode) {
        try {
            return getPIDCoefficientsAsync(mode).get();
        } catch (ExecutionException | InterruptedException e) {
            e.printStackTrace();
        }
        return pidCoefficients.getRawValue();
    }

    public CompletableFuture<PIDCoefficients> getPIDCoefficientsAsync(RunMode mode) {
        CompletableFuture<PIDCoefficients> future = new CompletableFuture<>();
        if(pidCoefficients.isValid()){
            future.complete(pidCoefficients.getValue());
        }else{
            PhotonLynxGetMotorPIDControlLoopCoefficientsCommand command = new PhotonLynxGetMotorPIDControlLoopCoefficientsCommand(hal.getLynxModule(), port, mode);
            hal.write(command);
            try {
                future = command.getResponse().thenApply(message -> {
                    LynxGetMotorPIDControlLoopCoefficientsResponse response = (LynxGetMotorPIDControlLoopCoefficientsResponse) message;
                    pidCoefficients.invalidate();

                    PIDCoefficients coefficients =  new PIDCoefficients(
                            LynxSetMotorPIDControlLoopCoefficientsCommand.externalCoefficientFromInternal(response.getP()),
                            LynxSetMotorPIDControlLoopCoefficientsCommand.externalCoefficientFromInternal(response.getI()),
                            LynxSetMotorPIDControlLoopCoefficientsCommand.externalCoefficientFromInternal(response.getD())
                    );

                    pidCoefficients.updateValue(coefficients);
                    return coefficients;
                });

                return future;
            } catch (LynxNackException e) {
                e.printStackTrace();
            }
        }
        return future;
    }

    @Override
    public PIDFCoefficients getPIDFCoefficients(RunMode mode) {
        if(pidfCoefficients.isValid()){
            return pidfCoefficients.getValue();
        }else{
            PhotonLynxGetMtorPIDFControlLoopCoefficientsCommand command = new PhotonLynxGetMtorPIDFControlLoopCoefficientsCommand(hal.getLynxModule(), port, mode);
            hal.write(command);
            try {
                LynxMessage message = command.getResponse().get(1, TimeUnit.SECONDS);
                LynxGetMotorPIDFControlLoopCoefficientsResponse response = (LynxGetMotorPIDFControlLoopCoefficientsResponse) message;
                pidfCoefficients.invalidate();

                PIDFCoefficients coefficients =  new PIDFCoefficients(
                        LynxSetMotorPIDControlLoopCoefficientsCommand.externalCoefficientFromInternal(response.getP()),
                        LynxSetMotorPIDControlLoopCoefficientsCommand.externalCoefficientFromInternal(response.getI()),
                        LynxSetMotorPIDControlLoopCoefficientsCommand.externalCoefficientFromInternal(response.getD()),
                        LynxSetMotorPIDControlLoopCoefficientsCommand.externalCoefficientFromInternal(response.getF()),
                        response.getInternalMotorControlAlgorithm().toExternal()
                );

                pidfCoefficients.updateValue(coefficients);
                return coefficients;
            } catch (ExecutionException | InterruptedException | TimeoutException | LynxNackException e) {
                e.printStackTrace();
            }
        }
        return pidfCoefficients.getRawValue();
    }

    @Override
    public void setTargetPositionTolerance(int tolerance) {
        this.positionTolerance = tolerance;
    }

    @Override
    public int getTargetPositionTolerance() {
        return positionTolerance;
    }

    public CompletableFuture<Double> getCurrentAsync(CurrentUnit unit) {
        PhotonLynxGetADCCommand command = new PhotonLynxGetADCCommand(hal.getLynxModule(), LynxGetADCCommand.Channel.motorCurrent(port), LynxGetADCCommand.Mode.ENGINEERING);
        try {
            hal.write(command);
            return command.getResponse().thenApply(result -> {
                LynxGetADCResponse response = (LynxGetADCResponse) result;
                return unit.convert(response.getValue(), CurrentUnit.MILLIAMPS);
            });
        } catch (LynxNackException e) {
            return null;
        }
    }

    @Override
    public double getCurrent(CurrentUnit unit) {
        try {
            return getCurrentAsync(unit).get();
        } catch (ExecutionException | InterruptedException e) {
            return -1;
        }
    }

    public CompletableFuture<Double> getCurrentAlertAsync(CurrentUnit unit) {
        if(currentLimit.isValid()){
            CompletableFuture<Double> future = new CompletableFuture<>();
            future.complete(unit.convert(currentLimit.getValue(), CurrentUnit.MILLIAMPS));
            return future;
        }
        PhotonLynxGetMotorChannelCurrentAlertLevelCommand command = new PhotonLynxGetMotorChannelCurrentAlertLevelCommand(hal.getLynxModule(), port);
        try {
            hal.write(command);
            return command.getResponse().thenApply(result -> {
                LynxGetMotorChannelCurrentAlertLevelResponse response = (LynxGetMotorChannelCurrentAlertLevelResponse) result;
                currentLimit.setValue((double) response.getCurrentLimit());
                return unit.convert(response.getCurrentLimit(), CurrentUnit.MILLIAMPS);
            });
        } catch (LynxNackException e) {
            e.printStackTrace();
        }

        CompletableFuture<Double> future = new CompletableFuture<>();
        future.complete(unit.convert(currentLimit.getRawValue(), CurrentUnit.MILLIAMPS));
        return future;
    }

    @Override
    public double getCurrentAlert(CurrentUnit unit) {
        try {
            return getCurrentAlertAsync(unit).get();
        } catch (ExecutionException | InterruptedException e) {
            return 0;
        }
    }

    @Override
    public void setCurrentAlert(double current, CurrentUnit unit) {
        PhotonLynxSetMotorChannelCurrentAlertLevelCommand command = new PhotonLynxSetMotorChannelCurrentAlertLevelCommand(hal.getLynxModule(), port, (int) unit.toMilliAmps(current));
        hal.write(command);
    }

    @Override
    public boolean isOverCurrent() {
        return hal.getBulkData().isOverCurrent(port);
    }

    /**
     * WARNING. NOT IMPLEMENTED.
     */
    @Deprecated
    @Override
    public MotorConfigurationType getMotorType() {
        return null;
    }

    /**
     * WARNING. NOT IMPLEMENTED
     */
    @Deprecated
    @Override
    public void setMotorType(MotorConfigurationType motorType) {

    }

    @Deprecated
    @Override
    public DcMotorController getController() {
        return null;
    }

    @Override
    public int getPortNumber() {
        return port;
    }

    @Override
    public void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior) {
        RunMode mode = getMode();
        PhotonLynxSetMotorChannelModeCommand command = new PhotonLynxSetMotorChannelModeCommand(hal.getLynxModule(), port, mode, zeroPowerBehavior);
        hal.write(command);
    }

    public CompletableFuture<ZeroPowerBehavior> getZeroPowerBehaviorAsync() {
        CompletableFuture<ZeroPowerBehavior> future = new CompletableFuture<>();
        if(!zeroPowerBehavior.isValid()){
            PhotonLynxGetMotorChannelModeCommand command = new PhotonLynxGetMotorChannelModeCommand(hal.getLynxModule(), port);
            try {
                hal.write(command);
                return command.getResponse().thenApply(result -> {
                    LynxGetMotorChannelModeResponse response = (LynxGetMotorChannelModeResponse) result;
                    zeroPowerBehavior.updateValue(response.getZeroPowerBehavior());
                    return response.getZeroPowerBehavior();
                });
            } catch (LynxNackException e) {
                e.printStackTrace();
                future.complete(ZeroPowerBehavior.UNKNOWN);
            }
        }else{
            future.complete(zeroPowerBehavior.getRawValue());
        }
        return future;
    }

    @Override
    public ZeroPowerBehavior getZeroPowerBehavior() {
        try {
            return getZeroPowerBehaviorAsync().get();
        } catch (ExecutionException | InterruptedException e) {
            return ZeroPowerBehavior.UNKNOWN;
        }
    }

    @Override
    public void setPowerFloat() {
        setZeroPowerBehavior(ZeroPowerBehavior.FLOAT);
        setPower(0);
    }

    public CompletableFuture<Boolean> getPowerFloatAsync(){
        return getZeroPowerBehaviorAsync().thenCombine(getPowerAsync(), (b, p) -> b == ZeroPowerBehavior.FLOAT && p == 0);
    }

    @Override
    public boolean getPowerFloat() {
        return getZeroPowerBehavior() == ZeroPowerBehavior.FLOAT && getPower() == 0;
    }

    @Override
    public void setTargetPosition(int position) {
        PhotonLynxSetMotorTargetPositionCommand command = new PhotonLynxSetMotorTargetPositionCommand(hal.getLynxModule(), port, position, positionTolerance);
        hal.write(command);
    }

    @Deprecated
    @Override
    public int getTargetPosition() {
        return 0;
    }

    @Deprecated
    @Override
    public boolean isBusy() {
        return false;
    }

    @Override
    public int getCurrentPosition() {
        return hal.getBulkData().getEncoder(port);
    }

    @Deprecated
    @Override
    public void setMode(RunMode mode) {

    }

    public CompletableFuture<RunMode> getModeAsync(){
        PhotonLynxGetMotorChannelModeCommand command = new PhotonLynxGetMotorChannelModeCommand(hal.getLynxModule(), port);
        try {
            hal.write(command);
            return command.getResponse().thenApply(result -> {
                LynxGetMotorChannelModeResponse response = (LynxGetMotorChannelModeResponse) result;
                return response.getMode();
            });
        } catch (LynxNackException e) {
            e.printStackTrace();
            return null;
        }
    }

    @Override
    public RunMode getMode() {
        try {
            return getModeAsync().get();
        } catch (ExecutionException | InterruptedException e) {
            return null;
        }
    }

    @Override
    public void setDirection(Direction direction) {
        this.direction = direction;
    }

    @Override
    public Direction getDirection() {
        return direction;
    }

    @Override
    public void setPower(double power) {
        RunMode mode = getMode();

        power *= (direction == Direction.REVERSE) ? -1 : 1;

        switch (mode) {
            case RUN_TO_POSITION:
            case RUN_USING_ENCODER:
                power = Math.max(0, Math.min(1, power));
                double setPower = power * 2800;
                PhotonLynxSetMotorTargetVelocityCommand command = new PhotonLynxSetMotorTargetVelocityCommand(hal.getLynxModule(), port, (int) setPower);
                hal.write(command);
                break;
            case RUN_WITHOUT_ENCODER:
                power = Math.max(-1, Math.min(1, power));

                double setPower2 = ((power + 1) / 2) * (LynxSetMotorConstantPowerCommand.apiPowerLast - LynxSetMotorConstantPowerCommand.apiPowerFirst);
                setPower2 += LynxSetMotorConstantPowerCommand.apiPowerFirst;

                PhotonLynxSetMotorConstantPowerCommand command1 = new PhotonLynxSetMotorConstantPowerCommand(hal.getLynxModule(), port, (int) setPower2);
                hal.write(command1);

                break;
            case STOP_AND_RESET_ENCODER:
                break;
        }
    }
    @Deprecated
    public CompletableFuture<Double> getPowerAsync() {
        return null;
    }
    @Deprecated
    @Override
    public double getPower() {
        return 0;
    }
    @Deprecated
    @Override
    public Manufacturer getManufacturer() {
        return null;
    }
    @Deprecated
    @Override
    public String getDeviceName() {
        return null;
    }
    @Deprecated
    @Override
    public String getConnectionInfo() {
        return null;
    }
    @Deprecated
    @Override
    public int getVersion() {
        return 0;
    }
    @Deprecated
    @Override
    public void resetDeviceConfigurationForOpMode() {

    }
    @Deprecated
    @Override
    public void close() {

    }
}

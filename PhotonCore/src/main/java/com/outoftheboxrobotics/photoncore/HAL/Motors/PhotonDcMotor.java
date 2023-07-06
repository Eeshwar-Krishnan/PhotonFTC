package com.outoftheboxrobotics.photoncore.HAL.Motors;

import com.outoftheboxrobotics.photoncore.HAL.HAL;
import com.outoftheboxrobotics.photoncore.HAL.Motors.Commands.PhotonLynxGetMotorChannelEnableCommand;
import com.outoftheboxrobotics.photoncore.HAL.Motors.Commands.PhotonLynxGetMotorPIDControlLoopCoefficientsCommand;
import com.outoftheboxrobotics.photoncore.HAL.Motors.Commands.PhotonLynxSetMotorChannelEnableCommand;
import com.outoftheboxrobotics.photoncore.HAL.Motors.Commands.PhotonLynxSetMotorPIDFControlLoopCoefficientsCommand;
import com.outoftheboxrobotics.photoncore.HAL.Motors.Commands.PhotonLynxSetMotorTargetVelocityCommand;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.hardware.lynx.commands.LynxCommand;
import com.qualcomm.hardware.lynx.commands.LynxMessage;
import com.qualcomm.hardware.lynx.commands.core.LynxGetMotorChannelEnableResponse;
import com.qualcomm.hardware.lynx.commands.core.LynxGetMotorPIDControlLoopCoefficientsCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetMotorPIDControlLoopCoefficientsResponse;
import com.qualcomm.hardware.lynx.commands.core.LynxSetMotorPIDControlLoopCoefficientsCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxSetMotorPIDFControlLoopCoefficientsCommand;
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

    protected Direction direction = Direction.FORWARD;

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
        if(pidCoefficients.isValid()){
            return pidCoefficients.getValue();
        }else{
            PhotonLynxGetMotorPIDControlLoopCoefficientsCommand command = new PhotonLynxGetMotorPIDControlLoopCoefficientsCommand(hal.getLynxModule(), port, mode);
            hal.write(command);
            try {
                LynxMessage message = command.getResponse().get(1, TimeUnit.SECONDS);
                LynxGetMotorPIDControlLoopCoefficientsResponse response = (LynxGetMotorPIDControlLoopCoefficientsResponse) message;
                pidCoefficients.invalidate();

                PIDCoefficients coefficients =  new PIDCoefficients(
                        LynxSetMotorPIDControlLoopCoefficientsCommand.externalCoefficientFromInternal(response.getP()),
                        LynxSetMotorPIDControlLoopCoefficientsCommand.externalCoefficientFromInternal(response.getI()),
                        LynxSetMotorPIDControlLoopCoefficientsCommand.externalCoefficientFromInternal(response.getD())
                );

                pidCoefficients.updateValue(coefficients);
                return coefficients;
            } catch (ExecutionException | InterruptedException | TimeoutException | LynxNackException e) {
                e.printStackTrace();
            }
        }
        return pidCoefficients.getRawValue();
    }

    @Override
    public PIDFCoefficients getPIDFCoefficients(RunMode mode) {
        return null;
    }

    @Override
    public void setTargetPositionTolerance(int tolerance) {

    }

    @Override
    public int getTargetPositionTolerance() {
        return 0;
    }

    @Override
    public double getCurrent(CurrentUnit unit) {
        return 0;
    }

    @Override
    public double getCurrentAlert(CurrentUnit unit) {
        return 0;
    }

    @Override
    public void setCurrentAlert(double current, CurrentUnit unit) {

    }

    @Override
    public boolean isOverCurrent() {
        return false;
    }

    @Override
    public MotorConfigurationType getMotorType() {
        return null;
    }

    @Override
    public void setMotorType(MotorConfigurationType motorType) {

    }

    @Override
    public DcMotorController getController() {
        return null;
    }

    @Override
    public int getPortNumber() {
        return 0;
    }

    @Override
    public void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior) {

    }

    @Override
    public ZeroPowerBehavior getZeroPowerBehavior() {
        return null;
    }

    @Override
    public void setPowerFloat() {

    }

    @Override
    public boolean getPowerFloat() {
        return false;
    }

    @Override
    public void setTargetPosition(int position) {

    }

    @Override
    public int getTargetPosition() {
        return 0;
    }

    @Override
    public boolean isBusy() {
        return false;
    }

    @Override
    public int getCurrentPosition() {
        return 0;
    }

    @Override
    public void setMode(RunMode mode) {

    }

    @Override
    public RunMode getMode() {
        return null;
    }

    @Override
    public void setDirection(Direction direction) {

    }

    @Override
    public Direction getDirection() {
        return null;
    }

    @Override
    public void setPower(double power) {

    }

    @Override
    public double getPower() {
        return 0;
    }

    @Override
    public Manufacturer getManufacturer() {
        return null;
    }

    @Override
    public String getDeviceName() {
        return null;
    }

    @Override
    public String getConnectionInfo() {
        return null;
    }

    @Override
    public int getVersion() {
        return 0;
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {

    }

    @Override
    public void close() {

    }
}

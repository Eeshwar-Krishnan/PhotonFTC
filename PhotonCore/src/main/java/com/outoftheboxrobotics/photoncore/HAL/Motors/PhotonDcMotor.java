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
import com.outoftheboxrobotics.photoncore.HAL.PhotonHAL;
import com.outoftheboxrobotics.photoncore.hardware.PhotonLynxDcMotorController;
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
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
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

public class PhotonDcMotor extends DcMotorImplEx {
    private final int port;
    private PhotonHAL hal;
    private final LastKnown<Boolean> enabled;
    private final LastKnown<Integer> setVelocity;
    private final LastKnown<Double> velocity;
    private final LastKnown<PIDCoefficients> pidCoefficients;
    private final LastKnown<PIDFCoefficients> pidfCoefficients;
    private final LastKnown<Double> currentLimit;
    private final LastKnown<ZeroPowerBehavior> zeroPowerBehavior;

    protected Direction direction = Direction.FORWARD;

    protected int positionTolerance = 5;

    protected PhotonDcMotor(PhotonLynxDcMotorController controller, int port, PhotonHAL hal){
        super(controller, port);
        this.port = port;
        this.hal = hal;

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

    public double getCorrectedCurrent(CurrentUnit unit) {
        return getCurrent(unit) * Math.abs(getPower());
    }

    public CompletableFuture<Double> getCorrectedCurrentAsync(CurrentUnit unit) {
        PhotonLynxGetADCCommand command = new PhotonLynxGetADCCommand(hal.getLynxModule(), LynxGetADCCommand.Channel.motorCurrent(port), LynxGetADCCommand.Mode.ENGINEERING);
        try {
            hal.write(command);
            return command.getResponse().thenApply(result -> {
                LynxGetADCResponse response = (LynxGetADCResponse) result;
                return unit.convert(response.getValue() * Math.abs(getPower()), CurrentUnit.MILLIAMPS);
            });
        } catch (LynxNackException e) {
            return null;
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
}

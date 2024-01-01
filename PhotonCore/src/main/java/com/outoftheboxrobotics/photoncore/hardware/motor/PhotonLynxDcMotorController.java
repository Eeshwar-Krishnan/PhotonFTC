package com.outoftheboxrobotics.photoncore.hardware.motor;

import android.content.Context;

import com.outoftheboxrobotics.photoncore.hardware.motor.commands.PhotonLynxGetADCCommand;
import com.outoftheboxrobotics.photoncore.hardware.motor.commands.PhotonLynxGetMotorChannelCurrentAlertLevelCommand;
import com.outoftheboxrobotics.photoncore.hardware.motor.commands.PhotonLynxGetMotorChannelEnableCommand;
import com.outoftheboxrobotics.photoncore.hardware.motor.commands.PhotonLynxGetMotorChannelModeCommand;
import com.outoftheboxrobotics.photoncore.hardware.motor.commands.PhotonLynxGetMotorConstantPowerCommand;
import com.outoftheboxrobotics.photoncore.hardware.motor.commands.PhotonLynxGetMotorPIDControlLoopCoefficientsCommand;
import com.outoftheboxrobotics.photoncore.hardware.motor.commands.PhotonLynxGetMotorPIDFControlLoopCoefficientsCommand;
import com.outoftheboxrobotics.photoncore.hardware.motor.commands.PhotonLynxGetMotorTargetPositionCommand;
import com.outoftheboxrobotics.photoncore.hardware.motor.commands.PhotonLynxGetMotorTargetVelocityCommand;
import com.qualcomm.hardware.lynx.LynxDcMotorController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.hardware.lynx.LynxUnsupportedCommandException;
import com.qualcomm.hardware.lynx.LynxUsbUtil;
import com.qualcomm.hardware.lynx.commands.core.LynxGetADCResponse;
import com.qualcomm.hardware.lynx.commands.core.LynxGetMotorChannelCurrentAlertLevelResponse;
import com.qualcomm.hardware.lynx.commands.core.LynxGetMotorChannelEnableResponse;
import com.qualcomm.hardware.lynx.commands.core.LynxGetMotorChannelModeResponse;
import com.qualcomm.hardware.lynx.commands.core.LynxGetMotorConstantPowerResponse;
import com.qualcomm.hardware.lynx.commands.core.LynxGetMotorPIDControlLoopCoefficientsResponse;
import com.qualcomm.hardware.lynx.commands.core.LynxGetMotorPIDFControlLoopCoefficientsResponse;
import com.qualcomm.hardware.lynx.commands.core.LynxGetMotorTargetPositionResponse;
import com.qualcomm.hardware.lynx.commands.core.LynxGetMotorTargetVelocityResponse;
import com.qualcomm.hardware.lynx.commands.core.LynxSetMotorConstantPowerCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxSetMotorPIDControlLoopCoefficientsCommand;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

public class PhotonLynxDcMotorController extends LynxDcMotorController implements PhotonDcMotorController {

    public PhotonLynxDcMotorController(Context context, LynxModule module) throws RobotCoreException, InterruptedException {
        super(context, module);
    }

    @Override
    public double getMotorCorrectedCurrent(int motor, CurrentUnit unit) {

        return getMotorCorrectedCurrentAsync(motor,unit).join();
    }

    @Override
    public CompletableFuture<PIDCoefficients> getPIDCoefficientsAsync(int motor, DcMotor.RunMode mode) {

        PhotonLynxGetMotorPIDControlLoopCoefficientsCommand command = new PhotonLynxGetMotorPIDControlLoopCoefficientsCommand(getModule(),motor,mode);
        try {
            command.acquireNetworkLock();
            command.send();
            return command.getResponse().thenApply(message -> {
                try {
                    command.releaseNetworkLock();
                } catch (InterruptedException e) {
                    handleException(e);
                }
                LynxGetMotorPIDControlLoopCoefficientsResponse response = (LynxGetMotorPIDControlLoopCoefficientsResponse) message;
                return new PIDCoefficients(response.getP(), response.getI(), response.getD());
            });
        } catch (LynxNackException | InterruptedException e) {
            handleException(e);
        }

        return CompletableFuture.completedFuture(LynxUsbUtil.makePlaceholderValue(new PIDCoefficients()));
    }

    @Override
    public CompletableFuture<PIDFCoefficients> getPIDFCoefficientsAsync(int motor, DcMotor.RunMode mode) {
        PhotonLynxGetMotorPIDFControlLoopCoefficientsCommand command = new PhotonLynxGetMotorPIDFControlLoopCoefficientsCommand(getModule(), motor, mode);
        try{
            command.acquireNetworkLock();
            getModule().sendCommand(command);
            return command.getResponse().thenApply(message -> {
                try {
                    command.releaseNetworkLock();
                } catch (InterruptedException e) {
                    handleException(e);
                }
                LynxGetMotorPIDFControlLoopCoefficientsResponse response = (LynxGetMotorPIDFControlLoopCoefficientsResponse) message;
                return new PIDFCoefficients(
                        LynxSetMotorPIDControlLoopCoefficientsCommand.externalCoefficientFromInternal(response.getP()),
                        LynxSetMotorPIDControlLoopCoefficientsCommand.externalCoefficientFromInternal(response.getI()),
                        LynxSetMotorPIDControlLoopCoefficientsCommand.externalCoefficientFromInternal(response.getD()),
                        LynxSetMotorPIDControlLoopCoefficientsCommand.externalCoefficientFromInternal(response.getF()),
                        response.getInternalMotorControlAlgorithm().toExternal()
                );
            });
        }catch (LynxUnsupportedCommandException|InterruptedException e)
        {
            handleException(e);
        }
        return CompletableFuture.completedFuture(LynxUsbUtil.makePlaceholderValue(new PIDFCoefficients()));
    }

    @Override
    public CompletableFuture<Double> getMotorCorrectedCurrentAsync(int motor, CurrentUnit unit) {
        
        return getMotorCurrentAsync(motor, unit).thenApply(current -> current*getMotorPower(motor));
    }

    @Override
    public CompletableFuture<Double> getMotorCurrentAsync(int motor, CurrentUnit unit) {
        
        PhotonLynxGetADCCommand command = new PhotonLynxGetADCCommand(getModule(), PhotonLynxGetADCCommand.Channel.motorCurrent(motor), PhotonLynxGetADCCommand.Mode.ENGINEERING);
        try {
            command.acquireNetworkLock();
            getModule().sendCommand(command);
            return command.getResponse().thenApply(message -> {
                try {
                    command.releaseNetworkLock();
                } catch (InterruptedException e) {
                    handleException(e);
                }
                LynxGetADCResponse response = (LynxGetADCResponse) message;
                return unit.convert(response.getValue(), CurrentUnit.MILLIAMPS);
            });
        }catch (InterruptedException | LynxUnsupportedCommandException e)
        {
            handleException(e);
        }
        return CompletableFuture.completedFuture(LynxUsbUtil.makePlaceholderValue(0.0));
    }

    @Override
    public CompletableFuture<Double> getMotorCurrentAlertAsync(int motor, CurrentUnit unit) {
        PhotonLynxGetMotorChannelCurrentAlertLevelCommand command = new PhotonLynxGetMotorChannelCurrentAlertLevelCommand(getModule(), motor);
        try {
            command.acquireNetworkLock();
            getModule().sendCommand(command);
            command.getResponse().thenApply(message-> {
                try {
                    command.releaseNetworkLock();
                } catch (InterruptedException e) {
                    handleException(e);
                }
                LynxGetMotorChannelCurrentAlertLevelResponse response = (LynxGetMotorChannelCurrentAlertLevelResponse) message;
                double currentAlert = response.getCurrentLimit();
                return unit.convert(currentAlert, CurrentUnit.MILLIAMPS);
            });
        }catch (LynxUnsupportedCommandException | InterruptedException e)
        {
            handleException(e);
        }
        return CompletableFuture.completedFuture(LynxUsbUtil.makePlaceholderValue(0.0));
    }


    @Override
    public CompletableFuture<DcMotor.RunMode> getMotorModeAsync(int motor) {

        PhotonLynxGetMotorChannelModeCommand command = new PhotonLynxGetMotorChannelModeCommand(getModule(), motor);
        try {
            command.acquireNetworkLock();
            getModule().sendCommand(command);
            try {
                command.releaseNetworkLock();
            } catch (InterruptedException e) {
                handleException(e);
            }
            return command.getResponse().thenApply(message -> {
                LynxGetMotorChannelModeResponse response = (LynxGetMotorChannelModeResponse) message;
                return response.getMode();
            });
        } catch (LynxUnsupportedCommandException | InterruptedException e) {
            handleException(e);
        }
        return CompletableFuture.completedFuture(LynxUsbUtil.makePlaceholderValue(DcMotor.RunMode.RUN_WITHOUT_ENCODER));
    }

    @Override
    public CompletableFuture<Double> getMotorPowerAsync(int motor) {
        CompletableFuture<DcMotor.RunMode> runmode = getMotorModeAsync(motor);
        return runmode.thenCompose((mode) -> {
            switch (mode) {
                case RUN_TO_POSITION:
                case RUN_USING_ENCODER: {
                    PhotonLynxGetMotorTargetVelocityCommand command = new PhotonLynxGetMotorTargetVelocityCommand(this.getModule(), motor);
                    try {
                        command.acquireNetworkLock();
                        getModule().sendCommand(command);
                        return command.getResponse().thenApply(message -> {
                            try {
                                command.releaseNetworkLock();
                            } catch (InterruptedException e) {
                                handleException(e);
                            }
                            LynxGetMotorTargetVelocityResponse response = (LynxGetMotorTargetVelocityResponse) message;
                            int iVelocity = response.getVelocity();
                            double result = Math.signum(iVelocity) * Range.scale(Math.abs(iVelocity), 0, getDefaultMaxMotorSpeed(motor), 0, apiPowerLast);
                            return result;
                        });

                    } catch (InterruptedException | LynxUnsupportedCommandException e) {
                        handleException(e);
                    }
                }
                case RUN_WITHOUT_ENCODER:
                default:
                {
                    PhotonLynxGetMotorConstantPowerCommand command = new PhotonLynxGetMotorConstantPowerCommand(this.getModule(), motor);
                    try {
                        command.acquireNetworkLock();
                        getModule().sendCommand(command);
                        return command.getResponse().thenApply(message -> {
                            try {
                                command.releaseNetworkLock();
                            } catch (InterruptedException e) {
                                handleException(e);
                            }
                            LynxGetMotorConstantPowerResponse response = (LynxGetMotorConstantPowerResponse) message;
                            int iPower = response.getPower();
                            double result = Range.scale(iPower, LynxSetMotorConstantPowerCommand.apiPowerFirst, LynxSetMotorConstantPowerCommand.apiPowerLast, apiPowerFirst, apiPowerLast);
                            return result;
                        });
                    } catch (InterruptedException | LynxUnsupportedCommandException e) {
                        handleException(e);
                    }
                }
            }

            return CompletableFuture.completedFuture(LynxUsbUtil.makePlaceholderValue(0.0));
        });
    }

    @Override
    public CompletableFuture<DcMotor.ZeroPowerBehavior> getMotorZeroPowerBehaviorAsync(int motor) {
        PhotonLynxGetMotorChannelModeCommand command = new PhotonLynxGetMotorChannelModeCommand(getModule(), motor);
        try {
            command.acquireNetworkLock();
            getModule().sendCommand(command);
            return command.getResponse().thenApply(message -> {
                try {
                    command.releaseNetworkLock();
                } catch (InterruptedException e) {
                    handleException(e);
                }
                LynxGetMotorChannelModeResponse response = (LynxGetMotorChannelModeResponse) message;
                return response.getZeroPowerBehavior();
            });
        } catch (InterruptedException | LynxUnsupportedCommandException e) {
            handleException(e);
        }
        return CompletableFuture.completedFuture(LynxUsbUtil.makePlaceholderValue(DcMotor.ZeroPowerBehavior.BRAKE));
    }

    @Override
    public CompletableFuture<Boolean> getMotorPowerFloatAsync(int motor) {
        return getMotorZeroPowerBehaviorAsync(motor)
                .thenCompose(behavior ->
                        getMotorPowerAsync(motor)
                                .thenApply(power -> behavior == DcMotor.ZeroPowerBehavior.FLOAT && power < 1e-9)
                );
    }

    @Override
    public CompletableFuture<Integer> getMotorTargetPositionAsync(int motor) {
        PhotonLynxGetMotorTargetPositionCommand command = new PhotonLynxGetMotorTargetPositionCommand(getModule(), motor);
        try {
            command.acquireNetworkLock();
            getModule().sendCommand(command);
            return command.getResponse().thenApply(message -> {
                try {
                    command.releaseNetworkLock();
                } catch (InterruptedException e) {
                    handleException(e);
                }
                LynxGetMotorTargetPositionResponse response = (LynxGetMotorTargetPositionResponse) message;
                return (Integer) response.getTarget();
            });
        } catch (InterruptedException | LynxUnsupportedCommandException e) {
            handleException(e);
        }
        return CompletableFuture.completedFuture(LynxUsbUtil.makePlaceholderValue(0));
    }

    @Override
    public CompletableFuture<Boolean> isMotorEnabledAsync(int motor) {
        PhotonLynxGetMotorChannelEnableCommand command = new PhotonLynxGetMotorChannelEnableCommand(getModule(),motor);
        try {
            command.acquireNetworkLock();
            getModule().sendCommand(command);
            return command.getResponse().thenApply(message-> {
                try {
                    command.releaseNetworkLock();
                } catch (InterruptedException e) {
                    handleException(e);
                }
                LynxGetMotorChannelEnableResponse response = (LynxGetMotorChannelEnableResponse) message;
                boolean enabled = response.isEnabled();
                return enabled;
            });
        }catch (InterruptedException | LynxUnsupportedCommandException e)
        {
            handleException(e);
        }
        return CompletableFuture.completedFuture(LynxUsbUtil.makePlaceholderValue(true));
    }
}

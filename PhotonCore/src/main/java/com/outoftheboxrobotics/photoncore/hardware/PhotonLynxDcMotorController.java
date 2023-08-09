package com.outoftheboxrobotics.photoncore.hardware;

import android.content.Context;

import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxDcMotorController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.hardware.lynx.LynxUnsupportedCommandException;
import com.qualcomm.hardware.lynx.commands.LynxCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetMotorChannelModeCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetMotorChannelModeResponse;
import com.qualcomm.hardware.lynx.commands.core.LynxSetMotorChannelModeCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxSetMotorConstantPowerCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxSetMotorTargetVelocityCommand;
import com.qualcomm.hardware.lynx.commands.standard.LynxAck;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;
import com.qualcomm.robotcore.util.LastKnown;
import com.qualcomm.robotcore.util.Range;

/**
 * Optimized LynxDcMotorController class.
 */
public class PhotonLynxDcMotorController extends LynxDcMotorController {
    public PhotonLynxDcMotorController(Context context, LynxModule module) throws RobotCoreException, InterruptedException {
        super(context, module);
    }

    /**
     * Create last known values before any changes are issued to motors
     */
    @Override
    protected void doHook() {
        lastKnownIPower= LastKnown.createArray(LynxConstants.NUMBER_OF_MOTORS);
        lastKnownRunMode= LastKnown.createArray(LynxConstants.NUMBER_OF_MOTORS);
    }



    /** MOTOR POWER **/

    private LastKnown<Integer>[] lastKnownIPower;

    @Override
    public double getMotorPower(int motor) {
        if(PhotonCore.photon==null) return super.getMotorPower(motor);
        Integer iPower = lastKnownIPower[motor].getNonTimedValue();
        if(iPower==null)
            return 0.0;
        return Range.scale(iPower, LynxSetMotorConstantPowerCommand.apiPowerFirst, LynxSetMotorConstantPowerCommand.apiPowerLast, apiPowerFirst, apiPowerLast);
    }


    @Override
    public void setMotorPower(int motor, double apiMotorPower) {
        if(PhotonCore.photon==null) {
            super.setMotorPower(motor,apiMotorPower);
            return;
        }
        double power = Range.clip(apiMotorPower, apiPowerFirst, apiPowerLast);
        Integer iPower;
        DcMotor.RunMode mode = getMotorMode(motor);
        LynxCommand<LynxAck> command = null;
        switch (mode)
        {
            case RUN_TO_POSITION:
            case RUN_USING_ENCODER:
                // Scale 'power' to configured maximum motor speed. This is mostly for legacy
                // compatibility, as setMotorVelocity exposes this more directly.
                power = Math.signum(power) * Range.scale(Math.abs(power), 0, apiPowerLast, 0, getDefaultMaxMotorSpeed(motor));
                iPower = (int)power;
                if(PhotonCore.photon.consistentLoopTimes())
                    command = new LynxSetMotorTargetVelocityCommand(this.getModule(), motor, iPower);
                else if(!iPower.equals(lastKnownIPower[motor].getNonTimedValue()))
                {
                    lastKnownIPower[motor].setValue(iPower);
                    command = new LynxSetMotorTargetVelocityCommand(this.getModule(), motor, iPower);
                }
                break;

            case RUN_WITHOUT_ENCODER:

                power = Range.scale(power, apiPowerFirst, apiPowerLast, LynxSetMotorConstantPowerCommand.apiPowerFirst, LynxSetMotorConstantPowerCommand.apiPowerLast);
                iPower = (int)power;

                if(PhotonCore.photon.consistentLoopTimes())
                    command = new LynxSetMotorConstantPowerCommand(this.getModule(), motor, iPower);
                else if(!iPower.equals(lastKnownIPower[motor].getNonTimedValue()))
                {
                    lastKnownIPower[motor].setValue(iPower);
                    command = new LynxSetMotorConstantPowerCommand(this.getModule(), motor, iPower);
                }
                break;
        }
        try {
            if (command != null)
            {
                getModule().sendCommand(command);
            }
        }
        catch (InterruptedException | RuntimeException | LynxUnsupportedCommandException e)
        {
            handleException(e);
        }

    }

    /** RUN MODE **/

    private LastKnown<DcMotor.RunMode>[] lastKnownRunMode;

    @Override
    public DcMotor.RunMode getMotorMode(int motor) {
        if(PhotonCore.photon==null) return super.getMotorMode(motor);
        DcMotor.RunMode runMode = lastKnownRunMode[motor].getNonTimedValue();
        if(runMode==null)
        {
            LynxGetMotorChannelModeCommand command = new LynxGetMotorChannelModeCommand(getModule(), motor);
            try {
                LynxGetMotorChannelModeResponse response = command.sendReceive();
                lastKnownRunMode[motor].setValue(response.getMode());
                return response.getMode();
            } catch (InterruptedException | LynxNackException e) {
                handleException(e);
            }
        }
        return runMode;
    }

    @Override
    public void setMotorMode(int motor, DcMotor.RunMode mode) {
        if(PhotonCore.photon==null)
        {
            super.setMotorMode(motor, mode);
            return;
        }
        DcMotor.RunMode runMode = lastKnownRunMode[motor].getNonTimedValue();
        LynxCommand<LynxAck> command = null;
        if(runMode!=null)
        {

            if(PhotonCore.photon.consistentLoopTimes())
            {
                command = new LynxSetMotorChannelModeCommand(getModule(), motor, mode, getMotorZeroPowerBehavior(motor));

            }else if(runMode!=mode)
            {
                command = new LynxSetMotorChannelModeCommand(getModule(), motor, mode, getMotorZeroPowerBehavior(motor));
            }
        }else command =new LynxSetMotorChannelModeCommand(getModule(), motor, mode, getMotorZeroPowerBehavior(motor));

        if(command!=null)
        {
            try {
                getModule().sendCommand(command);
            } catch (InterruptedException | LynxUnsupportedCommandException e) {
                handleException(e);
            }
        }
        lastKnownRunMode[motor].setValue(mode);

    }


}

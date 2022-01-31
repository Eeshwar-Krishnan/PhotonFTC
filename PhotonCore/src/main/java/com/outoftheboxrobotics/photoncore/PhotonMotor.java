package com.outoftheboxrobotics.photoncore;

import com.outoftheboxrobotics.photoncore.Commands.V2.LynxGetADCCommand;
import com.outoftheboxrobotics.photoncore.Commands.V2.LynxSetMotorChannelEnableCommand;
import com.outoftheboxrobotics.photoncore.Commands.V2.LynxSetMotorConstantPowerCommand;
import com.outoftheboxrobotics.photoncore.Commands.V2.LynxSetMotorTargetVelocityCommand;
import com.qualcomm.hardware.lynx.LynxDcMotorController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.hardware.lynx.commands.core.LynxGetADCResponse;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class PhotonMotor extends DcMotorImplEx {
    private LynxModule module;
    int port;

    private Integer current = null;

    protected PhotonMotor(LynxModule module, LynxDcMotorController controller, int port){
        super(controller, port);
        this.module = module;
        this.port = port;
    }

    @Override
    public synchronized void setPower(double power) {
        RunMode mode = getMode();
        int iPower = 0;
        switch (mode){
            case RUN_WITHOUT_ENCODER:
                power = Range.scale(power, LynxDcMotorController.apiPowerFirst, LynxDcMotorController.apiPowerLast, LynxSetMotorConstantPowerCommand.apiPowerFirst, LynxSetMotorConstantPowerCommand.apiPowerLast);
                iPower = (int)power;
                PhotonCore.registerSend(new LynxSetMotorConstantPowerCommand(module, port, iPower));
                PhotonCore.registerSend(new LynxSetMotorChannelEnableCommand(module, port, true));
                break;
            case RUN_TO_POSITION:
            case RUN_USING_ENCODER:
                power = Math.signum(power) * Range.scale(Math.abs(power), 0, LynxDcMotorController.apiPowerLast, 0, controller.getMotorType(port).getAchieveableMaxTicksPerSecondRounded());
                iPower = (int)power;
                PhotonCore.registerSend(new LynxSetMotorTargetVelocityCommand(module, port, iPower));
                PhotonCore.registerSend(new LynxSetMotorChannelEnableCommand(module, port, true));
                break;
            case STOP_AND_RESET_ENCODER:
                break;
        }
    }

    @Override
    public synchronized void setVelocity(double angularRate) {
        switch (getMode()) {
            case RUN_USING_ENCODER:
            case RUN_TO_POSITION:
                break;
            default:
                setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        int iTicksPerSecond = Range.clip((int)Math.round(angularRate),
                com.qualcomm.hardware.lynx.commands.core.LynxSetMotorTargetVelocityCommand.apiVelocityFirst,
                com.qualcomm.hardware.lynx.commands.core.LynxSetMotorTargetVelocityCommand.apiVelocityLast);

        PhotonCore.registerSend(new LynxSetMotorTargetVelocityCommand(module, port, iTicksPerSecond));
        PhotonCore.registerSend(new LynxSetMotorChannelEnableCommand(module, port, true));
    }

    @Override
    public double getCurrent(CurrentUnit unit) {
        LynxGetADCCommand command = new LynxGetADCCommand(module, LynxGetADCCommand.Channel.motorCurrent(port), LynxGetADCCommand.Mode.ENGINEERING);
        command.registerCallback(response -> current = ((LynxGetADCResponse)response).getValue());

        PhotonCore.registerGet(command);
        if(current == null){
            return 0;
        }
        return unit.convert(current, CurrentUnit.MILLIAMPS);
    }
}

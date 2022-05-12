package com.outoftheboxrobotics.photoncore.Testing;

import com.qualcomm.hardware.lynx.LynxAnalogInputController;
import com.qualcomm.hardware.lynx.LynxDcMotorController;
import com.qualcomm.hardware.lynx.LynxDigitalChannelController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxServoController;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelImpl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.ServoConfigurationType;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.util.HashMap;

public class SmartLynxModule {
    private LynxModule module;

    private LynxDcMotorController motorController;
    private LynxServoController servoController;
    private LynxAnalogInputController lynxAnalogInputController;
    private LynxDigitalChannelController lynxDigitalChannelController;

    private HashMap<Integer, DcMotorEx> cachedMotors;
    private HashMap<Integer, ServoImplEx> cachedServos;
    private HashMap<Integer, AnalogInput> cachedAI;
    private HashMap<Integer, DigitalChannel> cachedDC;

    public SmartLynxModule(LynxModule module) {
        try {
            motorController = new LynxDcMotorController(AppUtil.getDefContext(), module);
            servoController = new LynxServoController(AppUtil.getDefContext(), module);
            lynxAnalogInputController = new LynxAnalogInputController(AppUtil.getDefContext(), module);
            lynxDigitalChannelController = new LynxDigitalChannelController(AppUtil.getDefContext(), module);
        } catch (RobotCoreException | InterruptedException e) {
            e.printStackTrace();
        }

        cachedMotors = new HashMap<>();
        cachedServos = new HashMap<>();
        cachedAI = new HashMap<>();
        cachedDC = new HashMap<>();
    }

    public DcMotorEx getMotor(int port){
        if(!cachedMotors.containsKey(port)){
            cachedMotors.put(port, new DcMotorImplEx(motorController, port));
        }
        return cachedMotors.get(port);
    }

    public ServoImplEx getServo(int port){
        if(!cachedServos.containsKey(port)){
            cachedServos.put(port, new ServoImplEx(servoController, port, ServoConfigurationType.getStandardServoType()));
        }
        return cachedServos.get(port);
    }

    public AnalogInput getAnalogInput(int port){
        if(!cachedAI.containsKey(port)){
            cachedAI.put(port, new AnalogInput(lynxAnalogInputController, port));
        }
        return cachedAI.get(port);
    }

    public DigitalChannel getDigitalController(int port){
        if(!cachedDC.containsKey(port)){
            cachedDC.put(port, new DigitalChannelImpl(lynxDigitalChannelController, port));
        }
        return cachedDC.get(port);
    }
}
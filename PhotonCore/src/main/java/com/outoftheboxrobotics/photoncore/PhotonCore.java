package com.outoftheboxrobotics.photoncore;

import android.content.Context;

import com.outoftheboxrobotics.photoncore.hardware.PhotonLynxVoltageSensor;
import com.outoftheboxrobotics.photoncore.hardware.motor.PhotonDcMotor;
import com.outoftheboxrobotics.photoncore.hardware.motor.PhotonLynxDcMotorController;
import com.outoftheboxrobotics.photoncore.hardware.servo.PhotonCRServo;
import com.outoftheboxrobotics.photoncore.hardware.servo.PhotonLynxServoController;
import com.outoftheboxrobotics.photoncore.hardware.servo.PhotonServo;
import com.qualcomm.ftccommon.FtcEventLoop;
import com.qualcomm.hardware.lynx.LynxController;
import com.qualcomm.hardware.lynx.LynxDcMotorController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxModuleIntf;
import com.qualcomm.hardware.lynx.LynxServoController;
import com.qualcomm.hardware.lynx.LynxVoltageSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerImpl;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImpl;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.ConfigurationTypeManager;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.ServoConfigurationType;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.ftccommon.external.OnCreateEventLoop;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Set;
import java.util.concurrent.Semaphore;

public class PhotonCore implements OpModeManagerNotifier.Notifications {
    // Configuration and singleton values
    private static final String TAG = "PhotonCore";
    private static final PhotonCore instance = new PhotonCore();
    public static final boolean DEBUG=true;
    public static Photon photon;

    private OpModeManagerImpl opModeManager;

    public static int state = 0;

    private HashMap<LynxModuleIntf, Semaphore> mutexs;

    @SuppressWarnings({"unused"})
    @OnCreateEventLoop
    public static void attachEventLoop(Context context, FtcEventLoop eventLoop)
    {
        if(DEBUG) RobotLog.ii(TAG, "attachEventLoop: Attached PhotonCore to event loop");
        eventLoop.getOpModeManager().registerListener(instance);
        instance.opModeManager=eventLoop.getOpModeManager();
    }

    public static void acquire(LynxModuleIntf module) throws InterruptedException {
        instance.mutexs.get(module).acquire();
    }

    public static void release(LynxModuleIntf module) throws InterruptedException {
        instance.mutexs.get(module).release();
    }

    @Override
    public void onOpModePreInit(OpMode opMode) {
        if(opModeManager.getActiveOpModeName().equals(OpModeManager.DEFAULT_OP_MODE_NAME)) {
            state = 0;
            return;
        }

        state = 1;

        photon = opMode.getClass().getAnnotation(Photon.class);
        if(photon!=null) {
            RobotLog.ii(TAG, "PHOTON ACTIVATED");

            if(DEBUG) RobotLog.ii(TAG, "onOpModePreInit: Enabling PhotonCore optimizations for opMode %s", opModeManager.getActiveOpModeName());

            HardwareMap hardwareMap = opMode.hardwareMap;
            // Get the names of devices using reflection
            Map<HardwareDevice, Set<String>> deviceNames = ReflectionUtils.getFieldValue(opMode.hardwareMap, "deviceNames");
            assert deviceNames!=null;

            // The first step is to replace all LynxModules with PhotonLynxModules

            // Variables used to keep track of these replacements
            //   Since the standard architecture is HardwareDevice(e.g. Motor) <-> LynxController <-> LynxModule,
            //   we first swap in the LynxControllers, followed by the HardwareDevices on a per LynxController basis
            for(LynxController device : hardwareMap.getAll(LynxController.class))
            {
                String deviceName=Objects.requireNonNull(deviceNames.get(device)).iterator().next();


                // Get the LynxModule currently used by the Controller
                LynxModule usedModule = ReflectionUtils.getFieldValue(device, "module");

                //DO NOT touch rs485 expansion hubs
                if(!usedModule.isParent())
                    continue;


                // We now swap every controller we have a optimized implementation for, catering to
                // the needs of each controller

                if(device instanceof LynxDcMotorController && !(device instanceof PhotonLynxDcMotorController))
                {
                    try {
                        PhotonLynxDcMotorController photonLynxDcMotorController = new PhotonLynxDcMotorController(hardwareMap.appContext, usedModule);
                        hardwareMap.remove(deviceName, device);
                        hardwareMap.dcMotorController.remove(deviceName);
                        hardwareMap.dcMotorController.put(deviceName, photonLynxDcMotorController);
                        hardwareMap.put(deviceName, photonLynxDcMotorController);
                        List<DcMotor> motors = hardwareMap.getAll(DcMotorImpl.class);
                        for(DcMotor motor:motors)
                        {
                            if(motor.getController()==device)
                            {
                                String motorName = Objects.requireNonNull(deviceNames.get(motor)).iterator().next();
                                hardwareMap.remove(motorName, motor);
                                hardwareMap.dcMotor.remove(motorName);

                                MotorConfigurationType motorType = new MotorConfigurationType(ConfigurationTypeManager.class, motor.getMotorType().getXmlTag(), ConfigurationTypeManager.ClassSource.APK);
                                motorType.setTicksPerRev(motor.getMotorType().getTicksPerRev());
                                motorType.setGearing(motor.getMotorType().getGearing());
                                motorType.setMaxRPM(motor.getMotorType().getMaxRPM());
                                motorType.setOrientation(motor.getMotorType().getOrientation());
                                motorType.setAchieveableMaxRPMFraction(motor.getMotorType().getAchieveableMaxRPMFraction());

                                PhotonDcMotor photonDcMotor = new PhotonDcMotor(photonLynxDcMotorController, motor.getPortNumber(), motor.getDirection(), motorType);

                                hardwareMap.dcMotor.put(motorName, photonDcMotor);
                                hardwareMap.put(motorName, photonDcMotor);
                            }

                        }

                        device=photonLynxDcMotorController;
                    } catch (RobotCoreException | InterruptedException ignored) {

                    }
                }

                if(device instanceof LynxServoController && !(device instanceof PhotonLynxServoController))
                {

                    try {
                        PhotonLynxServoController photonLynxServoController = new PhotonLynxServoController(hardwareMap.appContext, usedModule);
                        hardwareMap.remove(deviceName, device);
                        hardwareMap.servoController.remove(deviceName);
                        hardwareMap.put(deviceName, photonLynxServoController);
                        hardwareMap.servoController.put(deviceName, photonLynxServoController);
                        photonLynxServoController.initializeHardware();
                        List<Servo> servos = hardwareMap.getAll(Servo.class);
                        for(Servo servo:servos)
                        {
                            if(servo.getController()==device)
                            {
                                String servoName=Objects.requireNonNull(deviceNames.get(servo)).iterator().next();
                                hardwareMap.remove(servoName, servo);
                                hardwareMap.servo.remove(servoName);

                                ServoConfigurationType type = new ServoConfigurationType();
                                //type.set

                                PhotonServo photonServo = new PhotonServo(photonLynxServoController, servo.getPortNumber());
                                hardwareMap.put(servoName, photonServo);
                                hardwareMap.servo.put(servoName, photonServo);
                            }

                        }

                        for(CRServo servo:hardwareMap.getAll(CRServoImpl.class))
                        {
                            if(servo.getController()==device)
                            {
                                String servoName=Objects.requireNonNull(deviceNames.get(servo)).iterator().next();
                                hardwareMap.remove(servoName, servo);
                                hardwareMap.crservo.remove(servoName);
                                PhotonCRServo photonServo = new PhotonCRServo(photonLynxServoController, servo.getPortNumber());
                                hardwareMap.put(servoName, photonServo);
                                hardwareMap.crservo.put(servoName, photonServo);
                            }

                        }
                        device=photonLynxServoController;
                    } catch (RobotCoreException | InterruptedException ignored) {

                    }
                }

                if(device instanceof LynxVoltageSensor)
                {
                    try {
                        PhotonLynxVoltageSensor photonLynxVoltageSensor=new PhotonLynxVoltageSensor(hardwareMap.appContext, usedModule);
                        hardwareMap.remove(deviceName, device);
                        hardwareMap.put(deviceName, photonLynxVoltageSensor);
                        device=photonLynxVoltageSensor;

                        // Bad hardware map architecture means we must babysit the separate values
                        hardwareMap.voltageSensor.remove(deviceName);
                        hardwareMap.voltageSensor.put(deviceName, photonLynxVoltageSensor);
                    } catch (RobotCoreException | InterruptedException ignored) {
                    }

                }
                ReflectionUtils.setFieldValue(device, usedModule, "module");
            }

            // I2C devices
            /**
            for(I2cDeviceSynchDeviceWithParameters device: hardwareMap.getAll(I2cDeviceSynchDeviceWithParameters.class))
            {
                String deviceName=Objects.requireNonNull(deviceNames.get(device)).iterator().next();
                // Get the LynxModule currently used by the I2C device
                LynxModule usedModule = ReflectionUtils.getFieldValue(device.getDeviceClient(), "module");
                if(!(usedModule instanceof PhotonLynxModule))
                {
                    usedModule=replacements.get(usedModule);
                }

                if(device instanceof BNO055IMUNew&& !(device instanceof PhotonBNO055IMUNew))
                {
                    int bus = ReflectionUtils.getFieldValue(device.getDeviceClient(), "bus");
                    PhotonI2cDeviceSynch deviceClient;
                    if(device.getDeviceClient() instanceof LynxI2cDeviceSynchV1)
                    {
                        deviceClient=new PhotonLynxI2cDeviceSynchV1((PhotonLynxModule) usedModule, bus);
                    }else
                    {
                        deviceClient=new PhotonLynxI2cDeviceSynchV2((PhotonLynxModule) usedModule, bus);
                    }
                    PhotonBNO055IMUNew photonBNO055IMUNew = new PhotonBNO055IMUNew(deviceClient, true);
                    hardwareMap.remove(deviceName,device);
                    hardwareMap.put(deviceName, photonBNO055IMUNew);
                }
                if(device instanceof BHI260IMU && !(device instanceof PhotonBHI260IMU))
                {
                    Integer bus = ReflectionUtils.getFieldValue(device.getDeviceClient(), "bus");
                    PhotonI2cDeviceSynch deviceClient;
                    if(device.getDeviceClient() instanceof LynxI2cDeviceSynchV1)
                    {
                        deviceClient=new PhotonLynxI2cDeviceSynchV1((PhotonLynxModule) usedModule, bus);
                    }else
                    {
                        deviceClient=new PhotonLynxI2cDeviceSynchV2((PhotonLynxModule) usedModule, bus);
                    }
                    PhotonBHI260IMU photonBNO055IMUNew = new PhotonBHI260IMU(deviceClient, true);
                    hardwareMap.remove(deviceName,device);
                    hardwareMap.put(deviceName, photonBNO055IMUNew);
                }
            }*/
        }
    }
    @Override
    public void onOpModePreStart(OpMode opMode) {}
    @Override
    public void onOpModePostStop(OpMode opMode) {
        state = 0;
    }
}

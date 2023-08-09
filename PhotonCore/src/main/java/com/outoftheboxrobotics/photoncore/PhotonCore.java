package com.outoftheboxrobotics.photoncore;

import android.content.Context;

import com.outoftheboxrobotics.photoncore.HAL.Motors.Commands.PhotonCommandBase;
import com.outoftheboxrobotics.photoncore.HAL.Motors.Commands.PhotonLynxGetBulkInputDataCommand;
import com.outoftheboxrobotics.photoncore.HAL.PhotonHAL;
import com.outoftheboxrobotics.photoncore.hardware.PhotonLynxDcMotorController;
import com.outoftheboxrobotics.photoncore.hardware.PhotonLynxModule;
import com.outoftheboxrobotics.photoncore.hardware.PhotonLynxServoController;
import com.outoftheboxrobotics.photoncore.hardware.PhotonLynxVoltageSensor;
import com.qualcomm.ftccommon.FtcEventLoop;
import com.qualcomm.hardware.lynx.LynxController;
import com.qualcomm.hardware.lynx.LynxDcMotorController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.hardware.lynx.LynxServoController;
import com.qualcomm.hardware.lynx.LynxUsbDevice;
import com.qualcomm.hardware.lynx.LynxVoltageSensor;
import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataResponse;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerImpl;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImpl;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.ftccommon.external.OnCreateEventLoop;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import java.util.Objects;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicLong;

public class PhotonCore implements Runnable, OpModeManagerNotifier.Notifications {
    // Configuration and singleton values
    private static final String TAG = "PhotonCore";
    private static final PhotonCore instance = new PhotonCore();
    private static final boolean DEBUG=true;
    public static Photon photon;

    private OpModeManagerImpl opModeManager;

    private final ArrayList<PhotonCommandBase> commandList;

    private PhotonHAL commandHal = null;

    private LynxGetBulkInputDataResponse commandBulkData = null;
    private final Object commandLock = new Object();
    private final Object supplierLock = new Object();

    private final ArrayList<PeriodicSupplier> suppliers;

    public PhotonCore(){
        commandList = new ArrayList<>();
        suppliers = new ArrayList<>();
    }

    public static PhotonHAL getControlHubHAL(){
        return instance.commandHal;
    }

    public static LynxGetBulkInputDataResponse getControlData() {
        synchronized (instance.commandLock) {
            return instance.commandBulkData;
        }
    }

    public static void start(HardwareMap map){
        for(LynxModule module : map.getAll(LynxModule.class)){
            if(module.isParent() && module.getSerialNumber() == LynxConstants.SERIAL_NUMBER_EMBEDDED){
                if(instance.commandHal == null){
                    instance.commandHal = new PhotonHAL(module);
                }
            }
        }
        PhotonLynxGetBulkInputDataCommand command = new PhotonLynxGetBulkInputDataCommand(instance.commandHal.getLynxModule());
        try {
            instance.commandBulkData = (LynxGetBulkInputDataResponse) command.getResponse().get();
        } catch (ExecutionException | InterruptedException | LynxNackException e) {
            e.printStackTrace();
        }
        Thread thread = new Thread(instance);
        thread.start();
    }

    public static void submit(PhotonCommandBase command){
        while(instance.commandList.size() > photon.maximumParallelCommands() && PhotonOpmodeWatchdog.getState() > 0);
        if(PhotonOpmodeWatchdog.getState() > 0){
            instance.commandList.add(command);
        }
    }

    @Override
    public void run() {
        AtomicBoolean controlInFlight = new AtomicBoolean(false);
        while(PhotonOpmodeWatchdog.getState() > 0){
            synchronized (instance.supplierLock) {
                for (PeriodicSupplier supplier : suppliers) {
                    supplier.update();
                }
            }
            commandList.removeIf((command) -> {
                try {
                    return command.getResponse().isDone();
                } catch (LynxNackException e) {
                    return true;
                }
            });
        }
    }


    @SuppressWarnings({"unused"})
    @OnCreateEventLoop
    public static void attachEventLoop(Context context, FtcEventLoop eventLoop)
    {
        if(DEBUG) RobotLog.ii(TAG, "attachEventLoop: Attached PhotonCore to event loop");
        eventLoop.getOpModeManager().registerListener(instance);
        instance.opModeManager=eventLoop.getOpModeManager();
    }

    @Override
    public void onOpModePreInit(OpMode opMode) {
        if(opModeManager.getActiveOpModeName().equals(OpModeManager.DEFAULT_OP_MODE_NAME))
            return;

        photon = opMode.getClass().getAnnotation(Photon.class);
        if(photon!=null) {

            if(DEBUG) RobotLog.ii(TAG, "onOpModePreInit: Enabling PhotonCore optimizations for opMode %s", opModeManager.getActiveOpModeName());

            HardwareMap hardwareMap=opMode.hardwareMap;
            // Get the names of devices using reflection
            Map<HardwareDevice, Set<String>> deviceNames = ReflectionUtils.getFieldValue(opMode.hardwareMap, "deviceNames");
            assert deviceNames!=null;

            // The first step is to replace all LynxModules with PhotonLynxModules

            // Variables used to keep track of these replacements
            Map<LynxModule, PhotonLynxModule> replacements = new HashMap<>();

            for(LynxModule lynxModule:hardwareMap.getAll(LynxModule.class))
            {
                if(lynxModule instanceof PhotonLynxModule) continue; // We have already replaced the LynxModule


                // Get name of LynxModule
                String deviceName= Objects.requireNonNull(deviceNames.get(lynxModule)).iterator().next();


                // Use reflection to initialize the replacement PhotonLynxModule
                PhotonLynxModule photonLynxModule = PhotonLynxModule.fromLynxModule(lynxModule);
                assert photonLynxModule!=null;


                // In order to swap xxx-LynxModules we need to re-link the LynxUsbDevice with the
                // PhotonLynxModule
                LynxUsbDevice lynxUsbDevice =photonLynxModule.getLynxUsbDevice().getDelegationTarget();
                lynxUsbDevice.removeConfiguredModule(lynxModule);


                // Since v8.20, the addConfiguredModule method was removed, so in order to achieve
                // the same thing, we need to use reflection
                ConcurrentHashMap<Integer, LynxModule> knownModules = (ReflectionUtils.getFieldValue(lynxUsbDevice, "knownModules"));
                assert knownModules != null;
                knownModules.put(photonLynxModule.getModuleAddress(), photonLynxModule);


                // Record the replacement
                replacements.put(lynxModule, photonLynxModule);


                // Swap the modules in the hardware map
                hardwareMap.remove(deviceName, lynxModule);
                hardwareMap.put(deviceName, photonLynxModule);

                photonLynxModule.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
            }

            // Since the standard architecture is HardwareDevice(e.g. Motor) <-> LynxController <-> LynxModule,
            // we first swap in the LynxControllers, followed by the HardwareDevices on a per LynxController basis
            for(LynxController device : hardwareMap.getAll(LynxController.class))
            {
                String deviceName=Objects.requireNonNull(deviceNames.get(device)).iterator().next();


                // Get the LynxModule currently used by the Controller
                LynxModule usedModule = ReflectionUtils.getFieldValue(device, "module");


                // We now swap every controller we have a optimized implementation for, catering to
                // the needs of each controller

                if(device instanceof LynxDcMotorController)
                {
                    try {
                        PhotonLynxDcMotorController PhotonLynxDcMotorController = new PhotonLynxDcMotorController(null, usedModule);
                        hardwareMap.remove(deviceName, device);
                        hardwareMap.put(deviceName, PhotonLynxDcMotorController);
                        for(DcMotor motor:hardwareMap.getAll(DcMotorImpl.class))
                        {
                            LynxDcMotorController controller = ReflectionUtils.getFieldValue(motor, "controller");
                            if(device.equals(controller))
                                ReflectionUtils.setFieldValue(motor, PhotonLynxDcMotorController, "controller");
                        }
                        device=PhotonLynxDcMotorController;
                    } catch (RobotCoreException | InterruptedException ignored) {

                    }

                }
                if(device instanceof LynxServoController)
                {
                    try {
                        PhotonLynxServoController PhotonLynxServoController = new PhotonLynxServoController(null, usedModule);
                        hardwareMap.remove(deviceName, device);
                        hardwareMap.put(deviceName, PhotonLynxServoController);
                        for(Servo servo:hardwareMap.getAll(ServoImpl.class))
                        {
                            LynxServoController controller =ReflectionUtils.getFieldValue(servo, "controller");
                            if(device.equals(controller))
                            {
                                ReflectionUtils.setFieldValue(servo, PhotonLynxServoController, "controller");
                            }
                        }
                        device=PhotonLynxServoController;
                    } catch (RobotCoreException | InterruptedException ignored) {

                    }
                }

                if(device instanceof LynxVoltageSensor)
                {
                    try {
                        PhotonLynxVoltageSensor PhotonLynxVoltageSensor=new PhotonLynxVoltageSensor(null, usedModule);
                        hardwareMap.remove(deviceName, device);
                        hardwareMap.put(deviceName, PhotonLynxVoltageSensor);
                        device=PhotonLynxVoltageSensor;

                        // Bad hardware map architecture means we must babysit the separate values
                        hardwareMap.voltageSensor.remove(deviceName);
                        hardwareMap.voltageSensor.put(deviceName, PhotonLynxVoltageSensor);
                    } catch (RobotCoreException | InterruptedException ignored) {
                    }

                }


                // We don't have this immediately after getting the used module, so we can benefit
                // from "free" recreations of hardware
                if(usedModule instanceof PhotonLynxModule) continue;


                ReflectionUtils.setFieldValue(device, replacements.get(usedModule), "module");
            }
        }
    }

    @Override
    public void onOpModePreStart(OpMode opMode) {}
    @Override
    public void onOpModePostStop(OpMode opMode) {}
}

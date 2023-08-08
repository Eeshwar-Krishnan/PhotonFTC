package com.outoftheboxrobotics.photoncore;

import com.outoftheboxrobotics.photoncore.HAL.HAL;
import com.outoftheboxrobotics.photoncore.HAL.Motors.Commands.PhotonCommandBase;
import com.outoftheboxrobotics.photoncore.HAL.Motors.Commands.PhotonLynxGetBulkInputDataCommand;
import com.outoftheboxrobotics.photoncore.HAL.PhotonHAL;
import com.qualcomm.hardware.lynx.LynxDcMotorController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataResponse;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;

import java.util.ArrayList;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicLong;

public class PhotonCore implements Runnable {
    protected static final PhotonCore instance = new PhotonCore();

    private final ArrayList<PhotonCommandBase> commandList;

    private PhotonHAL commandHal = null;

    private LynxGetBulkInputDataResponse commandBulkData = null;
    private final Object commandLock = new Object();
    private final Object supplierLock = new Object();

    private final ArrayList<PeriodicSupplier> suppliers;

    public static class ExperimentalParameters{
        private final AtomicBoolean singlethreadedOptimized = new AtomicBoolean(true);
        private final AtomicInteger maximumParallelCommands = new AtomicInteger(4);
        private final AtomicLong bulkDataUpdateMs = new AtomicLong(30);

        public void setSinglethreadedOptimized(boolean state){
            this.singlethreadedOptimized.set(state);
        }

        public boolean setMaximumParallelCommands(int maximumParallelCommands){
            if(maximumParallelCommands <= 0){
                return false;
            }
            this.maximumParallelCommands.set(maximumParallelCommands);
            return true;
        }

        public void setBulkDataUpdateMs(long updateMs){
            bulkDataUpdateMs.set(updateMs);
        }
    }

    public static ExperimentalParameters experimental = new ExperimentalParameters();

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
        while(instance.commandList.size() > experimental.maximumParallelCommands.get() && PhotonOpmodeWatchdog.getState() > 0);
        if(PhotonOpmodeWatchdog.getState() > 0){
            instance.commandList.add(command);
        }
    }

    @Override
    public void run() {
        long timer = System.currentTimeMillis() + experimental.bulkDataUpdateMs.get();
        AtomicBoolean controlInFlight = new AtomicBoolean(false);
        while(PhotonOpmodeWatchdog.getState() > 0){
            synchronized (instance.supplierLock) {
                for (PeriodicSupplier supplier : suppliers) {
                    supplier.update();
                }
            }
            if(System.currentTimeMillis() > timer){
                timer = System.currentTimeMillis() + experimental.bulkDataUpdateMs.get();

                /**
                 * ..and so he left, more steel and circuit than man
                if(commandHal != null && !controlInFlight.get()){
                    PhotonLynxGetBulkInputDataCommand command = new PhotonLynxGetBulkInputDataCommand(commandHal.getLynxModule());
                    try {
                        command.getResponse().thenApply((response) -> {
                            synchronized (commandLock) {
                                commandBulkData = (LynxGetBulkInputDataResponse) response;
                                controlInFlight.set(false);
                            }
                            return 1;
                        });
                        commandHal.write(command);
                        controlInFlight.set(true);
                    } catch (LynxNackException e) {
                        e.printStackTrace();
                    }
                }
                 */
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
}

package com.outoftheboxrobotics.photoncore

import android.content.Context
import com.outoftheboxrobotics.photoncore.Neutrino.Rev2MSensor.Rev2mDistanceSensorEx
import com.outoftheboxrobotics.photoncore.Neutrino.RevColorSensor.RevColorSensorV3Ex
import com.qualcomm.ftccommon.FtcEventLoop
import com.qualcomm.hardware.lynx.*
import com.qualcomm.hardware.lynx.commands.LynxCommand
import com.qualcomm.hardware.lynx.commands.LynxDatagram
import com.qualcomm.hardware.lynx.commands.LynxMessage
import com.qualcomm.hardware.lynx.commands.LynxRespondable
import com.qualcomm.hardware.lynx.commands.core.LynxSetMotorConstantPowerCommand
import com.qualcomm.hardware.lynx.commands.core.LynxSetServoPulseWidthCommand
import com.qualcomm.hardware.lynx.commands.standard.LynxAck
import com.qualcomm.hardware.rev.Rev2mDistanceSensor
import com.qualcomm.hardware.rev.RevColorSensorV3
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerImpl
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier.Notifications
import com.qualcomm.robotcore.exception.RobotCoreException
import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.hardware.I2cDeviceSynch
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple
import com.qualcomm.robotcore.hardware.configuration.LynxConstants
import com.qualcomm.robotcore.hardware.usb.RobotUsbDevice
import com.qualcomm.robotcore.util.RobotLog
import org.firstinspires.ftc.ftccommon.external.OnCreateEventLoop
import org.firstinspires.ftc.robotcore.internal.usb.exception.RobotUsbException
import java.util.concurrent.ConcurrentHashMap
import java.util.concurrent.atomic.AtomicBoolean
import java.util.concurrent.atomic.AtomicInteger

class PhotonCore : Runnable, Notifications {
    var enabled: AtomicBoolean
    protected var threadEnabled: AtomicBoolean
    private var modules: List<LynxModule>? = null
    private var thisThread: Thread? = null
    private var syncLock: Any? = null
    private val messageSync = Any()
    private var robotUsbDevice: RobotUsbDevice? = null
    private val usbDeviceMap: HashMap<LynxModule, RobotUsbDevice?>
    private var opModeManager: OpModeManagerImpl? = null

    class ExperimentalParameters {
        val singlethreadedOptimized = AtomicBoolean(true)
        val maximumParallelCommands = AtomicInteger(4)
        fun setSinglethreadedOptimized(state: Boolean) {
            singlethreadedOptimized.set(state)
        }

        fun setMaximumParallelCommands(maximumParallelCommands: Int): Boolean {
            if (maximumParallelCommands <= 0) {
                return false
            }
            this.maximumParallelCommands.set(maximumParallelCommands)
            return true
        }
    }

    init {
        enabled = AtomicBoolean(false)
        threadEnabled = AtomicBoolean(false)
        usbDeviceMap = HashMap()
    }

    private fun isSimilar(respondable1: LynxRespondable<*>, respondable2: LynxRespondable<*>): Boolean {
        return respondable1.destModuleAddress == respondable2.destModuleAddress && respondable1.commandNumber == respondable2.commandNumber
    }

    override fun run() {}

    private fun setLynxObject(device: Any, replacements: HashMap<LynxModule, PhotonLynxModule>) {
        val f = ReflectionUtils.getField(device.javaClass, LynxModule::class.java)
        if (f != null) {
            try {
                val module = f[device] as LynxModule
                if (replacements.containsKey(module)) {
                    f[device] = replacements[module]
                }
            } catch (e: IllegalAccessException) {
                e.printStackTrace()
            }
        }
    }

    override fun onOpModePreInit(opMode: OpMode) {
        if (opModeManager!!.activeOpModeName == OpModeManager.DEFAULT_OP_MODE_NAME) return;

        var replacedPrev = false
        var hasControlHub = false

        for (module in opMode.hardwareMap.getAll(LynxModule::class.java)) {
            if (module is PhotonLynxModule) {
                replacedPrev = true
            }
            if (LynxConstants.isEmbeddedSerialNumber(module.serialNumber)) {
                hasControlHub = true
            }
        }
        if (replacedPrev) {
            val toRemove = HashMap<String, HardwareDevice>()
            for (module in opMode.hardwareMap.getAll(LynxModule::class.java)) {
                if (module !is PhotonLynxModule) {
                    toRemove[opMode.hardwareMap.getNamesOf(module).toTypedArray()[0] as String] = module
                }
            }
            for (s in toRemove.keys) {
                opMode.hardwareMap.remove(s, toRemove[s])
            }
        } else {
            CONTROL_HUB = null
            EXPANSION_HUB = null
        }
        instance.modules = opMode.hardwareMap.getAll(LynxModule::class.java)
        // Module names of our lynx modules
        val moduleNames = ArrayList<String>()
        // Each module and its replacement class
        val replacements = HashMap<LynxModule, PhotonLynxModule>()

        val replacedNeutrino = HashMap<String, HardwareDevice>()
        val removedNeutrino = HashMap<String, HardwareDevice>()

        var usbDevice: LynxUsbDeviceImpl? = null

        for (module in (instance.modules as MutableList<LynxModule>?)!!) {
            moduleNames.add(opMode.hardwareMap.getNamesOf(module).toTypedArray()[0] as String)
        }

        for (lynxModName in moduleNames) {
            val lynxMod = opMode.hardwareMap.get(LynxModule::class.java, lynxModName) as LynxModule
            if (lynxMod !is PhotonLynxModule) return;

            try {
                // Create Our Lynx Module with data from the original Lynx
                val photonLynxModule = PhotonLynxModule((ReflectionUtils.getField(lynxMod.javaClass, "lynxUsbDevice")?.get(lynxMod) as LynxUsbDevice), (ReflectionUtils.getField(lynxMod.javaClass, "moduleAddress")?.get(lynxMod) as Int), (ReflectionUtils.getField(lynxMod.javaClass, "isParent")?.get(lynxMod) as Boolean), (ReflectionUtils.getField(lynxMod.javaClass, "isUserModule")?.get(lynxMod) as Boolean))
                // Print module name in warn log.
                RobotLog.ww("[PHOTON CORE LYNX NAMES]", lynxModName)
                // Copy all the data from the original Lynx to the new one.
                ReflectionUtils.deepCopy(lynxMod, photonLynxModule)
                // Set the new Lynx module to the hardware map.
                opMode.hardwareMap.put(lynxModName, photonLynxModule)
                // Marked it replaced
                replacements[lynxMod] = photonLynxModule
                if (lynxMod.isParent && hasControlHub && LynxConstants.isEmbeddedSerialNumber(lynxMod.serialNumber) && CONTROL_HUB == null) {
                    CONTROL_HUB = photonLynxModule
                    try {
                        val tmp = ReflectionUtils.getField(lynxMod.javaClass, "lynxUsbDevice")?.get(lynxMod) as LynxUsbDevice

                        usbDevice = if (tmp is LynxUsbDeviceDelegate) ReflectionUtils.getField(tmp.javaClass, "delegate")?.get(tmp) as LynxUsbDeviceImpl else tmp as LynxUsbDeviceImpl

                        // Get the USB and SyncLock from the corresponding files inside the usb.
                        robotUsbDevice = ReflectionUtils.getField(usbDevice!!.javaClass.superclass, "robotUsbDevice")?.get(usbDevice) as RobotUsbDevice
                        syncLock = ReflectionUtils.getField(usbDevice.javaClass, "engageLock")?.get(usbDevice)
                        usbDeviceMap[photonLynxModule] = robotUsbDevice
                    } catch (e: IllegalAccessException) { e.printStackTrace() }
                } else {
                    if (lynxMod.isParent) {
                        try {
                            val tmp = ReflectionUtils.getField(lynxMod.javaClass, "lynxUsbDevice")?.get(lynxMod) as LynxUsbDevice

                            usbDevice = if (tmp is LynxUsbDeviceDelegate) ReflectionUtils.getField(LynxUsbDeviceDelegate::class.java, "delegate")?.get(tmp) as LynxUsbDeviceImpl else tmp as LynxUsbDeviceImpl

                            robotUsbDevice = ReflectionUtils.getField(usbDevice!!.javaClass.superclass, "robotUsbDevice")?.get(usbDevice) as RobotUsbDevice
                            syncLock = ReflectionUtils.getField(usbDevice.javaClass, "engageLock")?.get(usbDevice)
                            usbDeviceMap[photonLynxModule] = robotUsbDevice
                        } catch (e: IllegalAccessException) { e.printStackTrace() }
                    }
                    EXPANSION_HUB = photonLynxModule
                }
            } catch (e: IllegalAccessException) { e.printStackTrace() }
        }
        for (m in replacements.keys) {
            usbDevice!!.removeConfiguredModule(m)
            try {
                usbDevice.addConfiguredModule(replacements[m])
            } catch (e: InterruptedException) { e.printStackTrace() } catch (e: RobotCoreException) { e.printStackTrace() }
        }
        // For Each Device
        for (device in opMode.hardwareMap.getAll(HardwareDevice::class.java)) {
            if (device !is LynxModule) {
                // Log device name
                RobotLog.i(opMode.hardwareMap.getNamesOf(device).toTypedArray()[0].toString())

                when(device) {
                    is I2cDeviceSynchDevice<*> -> {
                        try {
                            var device2 = ReflectionUtils.getField(device.javaClass, "deviceClient")?.get(device) as I2cDeviceSynchSimple

                            if (device2 !is LynxI2cDeviceSynch) {
                                device2 = ReflectionUtils.getField(device2.javaClass, "i2cDeviceSynchSimple")?.get(device2) as I2cDeviceSynchSimple
                            }
                            setLynxObject(device2, replacements)
                            RobotLog.e("" + (device2 is LynxI2cDeviceSynch))
                        } catch (ignored: Exception) { }
                    }
                    is I2cDeviceSynchSimple -> {
                        try {
                            setLynxObject(ReflectionUtils.getField(device.javaClass, "deviceClient")?.get(device) as I2cDeviceSynchSimple, replacements)
                        } catch (ignored: Exception) { }
                    }
                    else -> {
                        setLynxObject(device, replacements)
                    }
                }

                // Not in when statement because it can be both?
                if (device is Rev2mDistanceSensor) {
                    var tmp: I2cDeviceSynch?
                    var owned: Boolean
                    try {
                        tmp = ReflectionUtils.getField(device.javaClass, "deviceClient")?.get(device) as I2cDeviceSynch
                        owned = ReflectionUtils.getField(device.javaClass, "deviceClientIsOwned")?.get(device) as Boolean

                        replacedNeutrino[opMode.hardwareMap.getNamesOf(device).toTypedArray()[0] as String] = Rev2mDistanceSensorEx(tmp, owned)
                        removedNeutrino[opMode.hardwareMap.getNamesOf(device).toTypedArray()[0] as String] = device
                    } catch (e: IllegalAccessException) { e.printStackTrace() }
                }
                if (device is RevColorSensorV3) {
                    var owned: Boolean
                    try {
                        val tmp = ReflectionUtils.getField(device.javaClass, "deviceClient")?.get(device) as I2cDeviceSynchSimple
                        owned = ReflectionUtils.getField(device.javaClass, "deviceClientIsOwned")?.get(device) as Boolean

                        replacedNeutrino[opMode.hardwareMap.getNamesOf(device).toTypedArray()[0] as String] = RevColorSensorV3Ex(tmp, owned)
                        removedNeutrino[opMode.hardwareMap.getNamesOf(device).toTypedArray()[0] as String] = device
                    } catch (e: IllegalAccessException) { e.printStackTrace() }
                }
            }
        }
        // Replace the stuffs in the hardware map with the new stuffs.
        for (s in replacedNeutrino.keys) {
            opMode.hardwareMap.remove(s, removedNeutrino[s])
            opMode.hardwareMap.put(s, replacedNeutrino[s])
        }
        // IF we are not in a thread, just stick ourselvs in a new one!
        if (thisThread == null || !thisThread!!.isAlive) {
            thisThread = Thread(this)
            threadEnabled.set(true)
            thisThread!!.start()
        }
    }

    override fun onOpModePreStart(opMode: OpMode) {}
    override fun onOpModePostStop(opMode: OpMode) {
        enabled.set(false)
        threadEnabled.set(false)
    }

    companion object {
        val instance = PhotonCore()
        var CONTROL_HUB: LynxModule? = null
        var EXPANSION_HUB: LynxModule? = null
        var experimental = ExperimentalParameters()
        fun enable() {
            instance.enabled.set(true)
            if (CONTROL_HUB != null && CONTROL_HUB!!.bulkCachingMode == LynxModule.BulkCachingMode.OFF) {
                CONTROL_HUB!!.bulkCachingMode = LynxModule.BulkCachingMode.AUTO
            }
            if (EXPANSION_HUB != null && EXPANSION_HUB!!.bulkCachingMode == LynxModule.BulkCachingMode.OFF) {
                EXPANSION_HUB!!.bulkCachingMode = LynxModule.BulkCachingMode.AUTO
            }
        }

        fun disable() {
            instance.enabled.set(false)
        }

        @OnCreateEventLoop
        fun attachEventLoop(context: Context?, eventLoop: FtcEventLoop) {
            eventLoop.opModeManager.registerListener(instance)
            instance.opModeManager = eventLoop.opModeManager
        }

        @Throws(LynxUnsupportedCommandException::class, InterruptedException::class)
        fun registerSend(command: LynxCommand<*>): Boolean {
            val photonModule = command.module as PhotonLynxModule
            if (!instance.usbDeviceMap.containsKey(photonModule)) {
                return false
            }
            synchronized(instance.messageSync) {
                while (photonModule.unfinishedCommands.size > experimental.maximumParallelCommands.get()) {
                    //RobotLog.ee("PhotonCore", ((PhotonLynxModule)CONTROL_HUB).getUnfinishedCommands().size() + " | " + ((PhotonLynxModule)EXPANSION_HUB).getUnfinishedCommands().size());
                }
                if (!experimental.singlethreadedOptimized.get()) {
                    var noSimilar = false
                    while (!noSimilar) {
                        noSimilar = true
                        for (respondable in photonModule.unfinishedCommands.values) {
                            if (instance.isSimilar(respondable, command)) {
                                noSimilar = false
                            }
                        }
                    }
                }
                val messageNum = ReflectionUtils.getField(command.javaClass, "messageNumber")?.get(command) as Int
                command.messageNumber = messageNum
                try {
                    val datagram = LynxDatagram(command)
                    command.serialization = datagram
                    if (command.isAckable || command.isResponseExpected) {
                        photonModule.unfinishedCommands[command.messageNumber] = (command as LynxRespondable<*>)
                    }
                    val bytes = datagram.toByteArray()
                    var msLatency = 0.0
                    synchronized(instance.syncLock!!) {
                        val start = System.nanoTime()
                        instance.usbDeviceMap[photonModule]!!.write(bytes)
                        val stop = System.nanoTime()
                        msLatency = (stop - start) * 1.0e-6
                    }
                    //RobotLog.ii("PhotonCore", "Wrote " + bytes.length + " bytes " + photonModule.getUnfinishedCommands().size() + " | " + (msLatency));
                    if (shouldAckImmediately(command)) {
                        command.onAckReceived(LynxAck(photonModule, false))
                    }
                } catch (e: LynxUnsupportedCommandException) {
                    e.printStackTrace()
                } catch (e: RobotUsbException) {
                    e.printStackTrace()
                }
            }
            return true
        }

        fun shouldParallelize(command: LynxCommand<*>?): Boolean {
            return command is LynxSetMotorConstantPowerCommand ||
                    command is LynxSetServoPulseWidthCommand
        }

        protected fun shouldAckImmediately(command: LynxCommand<*>?): Boolean {
            return command is LynxSetMotorConstantPowerCommand ||
                    command is LynxSetServoPulseWidthCommand
        }

        fun getCacheResponse(command: LynxCommand<*>?): LynxMessage? {
            return null
        }
    }
}

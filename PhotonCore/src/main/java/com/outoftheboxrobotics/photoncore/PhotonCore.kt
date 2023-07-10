package com.outoftheboxrobotics.photoncore

import android.content.Context
import com.qualcomm.ftccommon.FtcEventLoop
import com.qualcomm.hardware.lynx.commands.LynxRespondable
import com.qualcomm.hardware.lynx.commands.LynxResponse
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerImpl
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier.Notifications
import org.firstinspires.ftc.ftccommon.external.OnCreateEventLoop
import java.util.concurrent.CompletableFuture
import java.util.concurrent.atomic.AtomicBoolean
import java.util.concurrent.atomic.AtomicInteger

open class PhotonCore : Runnable, Notifications {
    protected var enabled: AtomicBoolean = AtomicBoolean(false)
    private var threadEnabled: AtomicBoolean = AtomicBoolean(false)
    private val commandMap: HashMap<LynxRespondable<*>, CompletableFuture<LynxResponse<*>>> = HashMap()
    private var opModeManager: OpModeManagerImpl? = null

    class ExperimentalParameters {
        private val singlethreadedOptimized = AtomicBoolean(true)
        private val maximumParallelCommands = AtomicInteger(4)
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

    override fun run() {
        while (threadEnabled.get()) {
            if (enabled.get()) {
                for (respondable in commandMap.keys) {
                    if (respondable.hasBeenAcknowledged()) { }
                }
            }
            try {
                Thread.sleep(5)
            } catch (e: InterruptedException) {
                e.printStackTrace()
            }
        }
    }

    override fun onOpModePreInit(opMode: OpMode) {}
    override fun onOpModePreStart(opMode: OpMode) {}
    override fun onOpModePostStop(opMode: OpMode) {
        enabled.set(false)
        threadEnabled.set(false)
    }

    companion object {
        protected val instance = PhotonCore()
        var experimental = ExperimentalParameters()
        fun enable() {
            instance.enabled.set(true)
        }

        fun disable() {
            instance.enabled.set(false)
        }

        @OnCreateEventLoop
        fun attachEventLoop(context: Context?, eventLoop: FtcEventLoop) {
            eventLoop.opModeManager.registerListener(instance)
            instance.opModeManager = eventLoop.opModeManager
        }
    }
}
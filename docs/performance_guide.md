# Performance with PhotonCore

This document is dedicated to helping teams achieve the best possible performance out of their
robot code as it covers how to effectively improve loop times.

Note: This document makes references to [this](https://blog.eeshwark.com/robotblog/photonftc-basic-explanation),
because it properly explains how the internals of the FTC SDK work.

## The basics

### RS485 or USB 

Unless you absolutely have to use RS485, USB is the recommend way to connect the Control Hub to the
Expansion Hub(and if you are really paranoid about USB power surges, you can use USB Surge Protectors, 
which are legal as per RE14.a). The reasoning behind using USB is to increase bandwidth between the
Android board in the Control Hub and each Hub.

TODO: Add photo with graph for better explanation

### Enabling PhotonCore

This is pretty straightforward; if you enable PhotonCore, you get the speedup associated with it. 
PhotonCore does several things to speedup your loop times:
- Skips waiting for ACKs from usual commands
- Allows sending commands in parallel
- Stops unnecessary commands from being sent
- Gives access to data read by the SDK that is normally not accessible

Here is how you can enable PhotonCore:

```java
@Photon // <--
@TeleOp
class TestTeleOp extends LinearOpMode {
    // ...  
}
```

### Using the manual bulk caching mode

Using the auto bulk caching mode is an widely-known way of achieving speedup, since it enables reading 
motor encoder positions, digital and analog sensors with a single command. However, unless you have 
awareness of every place you read each motor position, digital and analog sensor, you can accidentally
request an useless second bulk read. 

Here is how you can enable and use the manual bulk cache mode:

```java
@TeleOp
class TestTeleOp extends LinearOpMode {
    @override
    void runOpMode()
    {
        //...
        List<LynxModule> modules = hardwareMap.getAll(LynxModule.class);
        for(LynxModule module: modules)
            modules.setBulkCachingMode(BulkCachingMode.MANUAL);
        //...
        waitForStart();
        while(opModeIsActive())
        {
            for(LynxModule module: modules)
                modules.clearBulkCache();
            
            // ...
        }
    }
}
```

## Advanced techniques

### Voltage sensor

When the FTC SDK display the voltage on the Driver Station, it uses the same interfaces we use. 
These reads happen at intervals of ~3s and cannot be disabled, so PhotonCore gives you access to
this normally inaccessible data.

Her is how you can get the voltage automatically read:
```java
@TeleOp
class TestTeleOp extends LinearOpMode {
    @override
    void runOpMode()
    {
        //...
        PhotonLynxVoltageSensor sensor = hardwareMap.getAll(PhotonLynxVoltageSensor.class).iterator().next();
        //...
        waitForStart();
        while(opModeIsActive())
        {
            double voltage = sensor.getCachedVoltage();
            
            // ...
        }
    }
}
```



### Interleaving the IMU

Interleaving the IMU device is a method discovered by a member of FTC team 19071, which achieves a
speedup of ~2ms when reading the IMU. It involves sending a command(most often a LynxGetBulkInputCommand),
between asking the IMU for data and requesting the response. This skips sending a useless request which would
just return IN PROGRESS.

Here is how you can interleave the IMU:

```java
@Photon // <--- Photon is required for this
@TeleOp
class TestTeleOp extends LinearOpMode {
    @override
    void runOpMode()
    {
        PhotonBHI260IMU imu = hardwareMap.get(PhotonBHI260IMU.class, "imu");
        // PhotonBNO055IMUNew imu = hardwareMap.get(PhotonBNO055IMUNew.class, "imu");
        
        //...
        
        List<PhotonLynxModule> modules = hardwareMap.getAll(PhotonLynxModule.class);
        
        PhotonLynxModule usedModule = modules.get(0); 
        // Depending on whether you use the IMU from the Control Hub or Expansion Hub you need to
        // change the number

        for(PhotonLynxModule module: modules)
            modules.setBulkCachingMode(BulkCachingMode.MANUAL); // Manual mode is required here
        
        //...
        
        waitForStart();
        
        while(opModeIsActive())
        {
            modules.get(1).clearBulkCache(); // Again, adjust number here based on which IMU you use
            
            imu.scheduleInterleavedCommand(new PhotonLynxGetBulkInputCommand(usedModule));
            YawPitchRollAngles angles = imu.getYawPitchRollAngles();
            usedModule.feedBulkData((LynxGetBulkInputDataResponse)imu.getResult());
            
            // ...
        }
    }
}

```



#### Side Note: Threading the IMU

Threading the IMU was a method used by some teams to speedup reading the IMU. It involved reading the IMU
in a separate thread and using the data in the main thread. This method brought a surprising amount of 
speedup to loop times. 

The way it achieved this is still not precisely known. This is mostly due to the unstable(and unsafe) 
nature of threads. There are however some theories as to how those speedups were achieved:
* Threading the IMU skips the unnecessary middle request from being sent. This would happen because
the OS scheduler would allow the main thread to send a command between the request for data and the
request for the response.
* It read the IMU less frequently and thus uses less time. This again would happen because the main thread would be scheduled 
more frequently, and thus get more access to the lock.
* The teams' loop time calculation code could be wrong(as in not include the time between the **end** and **start** of each loop).
An example of wrong time loop time calculation code:
```java
@TeleOp
class TestTeleOp extends LinearOpMode {
    @override
    void runOpMode()
    {
        startIMUThread();
        // ...
        
        while(opModeIsActive())
        {
            double startTime = System.nanoTime();
            
            // ...
            
            double endTime = System.nanoTime();
            telemetry.addData("hz", 1000000000 / (endTime-startTime));
            telemetry.update();
            
            // This is wrong because any thing that happens here is not included in the calculation
            
            readIMU100Times();
        
        }
    }
}
```

That being said, there's no way this method actually does anything new to achieve speedup, 
since the command actually needs to be sent to and received from the Hub,
and using it introduces the risk of reading the IMU multiple times in a single loop, thus wasting more
time instead of speeding up loop times.

If you want to use it, use it at your own risk.


## Super advanced use

### Maximum possible performance™


When trying to get the maximum possible performance™, the goal is to simply get as many commands, 
as fast as possible, to each hub. At this level, even imperfections in transmission speeds end up counting.

It is now known that sending commands to the Control Hub is faster than sending the Expansion Hub.
A PhotonCore enhanced with [consistentLoopTimes=true]() OpMode which reads the hub, and writes to 
every motor and servo on the hub achieves ~140hz on the Control Hub and ~50 hz on the Expansion Hub.

This means that in order to achieve the maximum possible performance™, each hub needs to have a separate
loop(in a separate thread), so that the Control Hub doesn't have to wait for the slower Expansion Hub.

Since multithreading is a difficult topic(even outside of FTC), that has many pitfalls, implementing
such a OpMode is left as homework for more advanced teams.




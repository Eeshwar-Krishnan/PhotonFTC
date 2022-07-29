# PhotonCore Overview

### What is PhotonCore?

PhotonCore is a collection of optimization systems meant to speed up loop times with little to no compromises.

### Why does this exist?

There are some things that can be sped up internally by squeezing some extra performance out of everything.

### How do I use this

Its very simple, just put

```java
PhotonCore.enable();
```

at the beginning of your opmode to turn on write optimizations.

PhotonCore also has the variables

```java
PhotonCore.CONTROL_HUB
PhotonCore.EXPANSION_HUB
```

to make finding LynxModule objects either. EXPANSION_HUB will be null if run with just one hub. If used with android phones, CONTROL_HUB will be the hub connected to the phone, and EXPANSION_HUB will be the one connected over RS485.
If used with two expansion hubs both connected over USB, the CONTROL_HUB and EXPANSION_HUB variables will be arbitrarily assigned, and may not be the same from run to run.

NOTE: At this time, hubs connected via RS485 cannot be parallelized by photon, however connecting the expansion hub to the control hub via USB will work. It is not recommended to connect two expansion hubs via usb to an android phone.

### Experimental

There are a few experimental variables that you can tweak.
**NOTE: Experimental variables are EXPERIMENTAL and may result in degraded or unstable behaviour if modified**

**setSinglethreadedOptimized**: set to TRUE if only using one thread, set to FALSE if you plan on accessing hardware from multiple threads. Using multiple threads is NOT recommended as it may cause instibility. Default: TRUE

**setMaximumParallelCommands**: sets the maximum number of parallel commands allowed at once. Input must be an integer greater then 1. Lower numbers can be more stable when multithreading hardware, **numbers greater then 8 may cause instability and crashes**. Default: 4

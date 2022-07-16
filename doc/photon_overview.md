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

to make finding LynxModule objects either (NOTE: EXPANSION_HUB will be null if run with just one hub)

### Experimental

There are a few experimental variables that you can tweak.
**NOTE: Experimental variables are EXPERIMENTAL and may result in degraded or unstable behaviour if modified**

**setSinglethreadedOptimized**: set to TRUE if only using one thread, set to FALSE if you plan on accessing hardware from multiple threads. Using multiple threads is NOT recommended. Default: TRUE

**setMaximumParallelCommands**: sets the maximum number of parallel commands allowed at once. Input must be an integer between 1 and 9 inclusive. Lower numbers can be more stable when multithreading hardware. Default: 8

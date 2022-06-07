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

at the beginning of your opmode to turn on write optimizations. In addition, you can tell Photon to cache certain data behind the scenes (like motor current data and bulk read data) in order to read them with minimal slowdowns

In order to do this, simply run

```java
PhotonCore.addCacheIntent(/*Cache Intent Here*/);
```

Currently, the following cache intents are supported

```java
BulkReadCacheIntent(runDelayMs, hub);
MotorCurrentCacheIntent(runDelayMs, hub, motorPort);
```

All cache intent constructors first take a value representing the minimum amount of time that should be waited between data retrieval attempts. The bigger this number, the slower it will update, but the less background resources it will use.
It is recommended to use 5ms delays as a minimum, and using longer delays, such as 30-50ms, for data that doesn't need to be rapidly updated (motor current, etc).NeutrinoI2C

The second value in all constructors is the hub to run on, PhotonCore has the variables

```java
PhotonCore.CONTROL_HUB
PhotonCore.EXPANSION_HUB
```

to easily get the hubs you can run on. Make sure to ONLY use hubs that are physically connected to avoid a crash!

[Example Opmode](https://github.com/Eeshwar-Krishnan/PhotonFTC/blob/main/PhotonCore/src/main/java/com/outoftheboxrobotics/photoncore/Testing/PhotonExample.java)

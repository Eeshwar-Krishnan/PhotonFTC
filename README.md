# PhotonFTC
NOTE: This code will ONLY work on the REV Robotics Expansion Hub or REV Robotics Control Hub. No Modern Robotics system is supported!

This project is an initiative to push the FIRST api to its absolute limits, and to **give access to functionality and methods not normally exposed to the user**

PhotonFTC is split up into different projects depending on what the project is attempting to do

Projects at a glance:
 - **NeutrinoI2C**
      - Speeding up, optimizing, and expanding stock I2C drivers including
           - Rev 2M Distance Sensor (supports asynchronous non-blocking reading and different accuracy modes)
 - **PhotonCore**
     - Speeding up and paralleling the FTC control system for faster loop times

**NOTE**: There is no cross dependency in PhotonFTC. That means you can install one project individually without the others, and you do NOT have to download everything if you do not want to.

## Documentation:
 - [Neutrino Overview](https://github.com/Eeshwar-Krishnan/PhotonFTC/blob/main/doc/neutrino_overview.md)

## Installation instructions :

Installation is simple. Just go to build.gradle in your TeamCode module and add the following

```
maven { url 'https://jitpack.io' }
```

in repositories.

Then, add the following depending on what you want to install

**NeutrinoI2C**

```
implementation 'com.github.OutoftheBoxFTC:EasyTensorflowAPI:v1.0.0'
```

in dependencies. Then run a gradle sync, and everything should download!

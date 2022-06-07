# PhotonFTC
[![](https://jitpack.io/v/Eeshwar-Krishnan/PhotonFTC.svg)](https://jitpack.io/#Eeshwar-Krishnan/PhotonFTC)

NOTE: This code will ONLY work on the REV Robotics Expansion Hub or REV Robotics Control Hub. No Modern Robotics system is supported!

This project is an initiative to push the FIRST api to its absolute limits, and to **give access to functionality and methods not normally exposed to the user**

PhotonFTC is split up into different projects depending on what the project is attempting to do

Projects at a glance:
 - **NeutrinoI2C**
      - Speeding up, optimizing, and expanding stock I2C drivers including
           - Rev 2M Distance Sensor (supports asynchronous non-blocking reading and different accuracy modes)
           - MB1242 Ultrasonic Sensor (Asynchronous sensing and auto-ping)
 - **PhotonCore - In Development: Not available yet (check dev branch to see progress)**
     - Speeding up and paralleling the FTC control system for faster loop times

**NOTE**: There is no cross dependency in PhotonFTC. That means you can install one project individually without the others, and you do NOT have to download everything if you do not want to.

## Documentation:
 - [Neutrino Overview](https://github.com/Eeshwar-Krishnan/PhotonFTC/blob/main/doc/neutrino_overview.md)

## Installation instructions :
Installation is simple. Just go to build.gradle in your Teamcode module and add the following under repositories

```
maven { url 'https://jitpack.io' }
```

Then, depending on which project you want to install, add the following under dependencies

**NeutrinoI2C**:
```
implementation 'com.github.Eeshwar-Krishnan.PhotonFTC:NeutrinoI2C:v1.0.5'
```

Then run a gradle sync, and everything should download!

## OnBotJava:
This library can be ported to OnBotJava, however I strongly recommend anyone using this library to use it in Android Studio to avoid a lot of pain that will come from porting it.

## Important Note:
This API is still a work in progress and is subject to change. The basic usage of the API will remain the same for the forseeable future, but new features will be added as teams share feedback on things they want to see. 

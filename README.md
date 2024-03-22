# PhotonFTC
[![](https://jitpack.io/v/Eeshwar-Krishnan/PhotonFTC.svg)](https://jitpack.io/#Eeshwar-Krishnan/PhotonFTC)

NOTE: This code will ONLY work on the REV Robotics Expansion Hub or REV Robotics Control Hub. No Modern Robotics system is supported!

This project is an initiative to push the FIRST api to its absolute limits, and to **give access to functionality and methods not normally exposed to the user**

## Documentation:
 - [PhotonCore Overview](https://photondocs.pages.dev/)

## Installation instructions :
Installation is simple. Just go to build.gradle in your Teamcode module and add the following under repositories

```
maven { url = 'https://jitpack.io' }
```

Then add the following under dependencies

```
implementation 'com.github.Eeshwar-Krishnan:PhotonFTC:v3.0.1-ALPHA'
```

Then run a gradle sync, and everything should download!

## OnBotJava:
This library can be ported to OnBotJava, however I strongly recommend anyone using this library to use it in Android Studio to avoid a lot of pain that will come from porting it.

## Important Note:
This API is still a work in progress and is subject to change. The basic usage of the API will remain the same for the forseeable future, but new features will be added as teams share feedback on things they want to see. 

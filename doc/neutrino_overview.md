# NeutrinoI2C Overview

### What is NeutrinoI2C?

NeutrinoI2C is a collection of optimized drivers for stock i2c devices.

### Why does this exist?

FIRST does a great job of creating new I2C drivers for teams to use, but I2C in FTC has some inherent limitations causing them to slow down. By specifically optimizing each driver by hand for the given device, we can squeeze out slightly better performance.NeutrinoI2C

In addition, there are some structural changes that can be made, like making the driver asynchronous so that it does not block user code from running, and we can also expose methods to the user that are otherwise hidden

# Devices

### Rev 2M Distance Sensor

Creating the 2m sensor object is easy, simply pass a Rev2mDistanceSensor object in the constructor

```java
Rev2mDistanceSensor sensor = hardwareMap.get(Rev2mDistanceSensor.class, "test");

AsyncRev2MSensor asyncSensor = new AsyncRev2MSensor(sensor);
```

from there, several methods are available

***getDistance(DistanceUnit unit)***: Gets the measured distance from the sensor. Method is non-blocking and returned data is NOT guaranteed to be new

***getLastMeasurementTimestamp()***: Returns the timestamp of the last time data was read from the sensor. Can be used to detect if new data is available

***hasNewData()*** returns true if new data has been acquired since the last call to getDistance

***setMeasurementIntervalMs(long interval)*** sets the amount of time between reads that the driver should wait. Beyond changing the update rate, this will actually change the accuracy of the sensor, with longer wait times giving more precise data. **NOTE: This value is overridden by setSensorAccuracyMode**

***setSensorAccuracyMode(AccuracyMode mode)*** sets the accuracy mode of the sensor. **THIS IS OVERRIDDEN BY setMeasurementIntervalMs**. The following presets are available:
MODE_HIGH_ACCURACY: 150ms delay between reads, gives the most precise values at a low speed
MODE_BALANCED: 75ms delay between reads, gives a good balance of speed and precision
MODE_HIGH_SPEED: 33ms delay between reads, gives the fastest reads and lowest precision. NOTE: This is the stock operation mode of the driver

***disable()***: Disables the device, stopping reads from occurring in the background
***enable()***: Enables the device, starting reads in the background again
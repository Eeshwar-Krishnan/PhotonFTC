# PhotonCore Documentation 📘

## Table of Contents
1. [Introduction](#introduction)
2. [Classes Overview](#classes-overview)
    - [PhotonDcMotor](#photondcmotor)
    - [PhotonCRServo](#photoncrservo)
    - [PhotonServo](#photonservo)
    - [PhotonLynxServoController](#photonlynxservocontroller)
    - [PhotonAdvancedDcMotor](#photonadvanceddcmotor)
    - [PhotonLynxCommandListener](#photonlynxcommandlistener)
3. [Installation Guide](#installation-guide)
4. [Usage Guide](#usage-guide)
5. [Conclusion](#conclusion)

## Introduction

**PhotonCore** is a library designed for FIRST Tech Challenge (FTC) robotics teams to provide advanced control over motors and servos. It enhances the functionality of the standard FTC SDK by adding asynchronous operations and advanced control mechanisms.

---

## Classes Overview

### PhotonDcMotor

- **Purpose**: This class extends the functionality of a DC motor by providing asynchronous operations.
- **Key Features**:
  - **Asynchronous Methods**: Allows fetching motor properties like current, power, and PID coefficients asynchronously.
  - **Enhanced Control**: Offers methods to get the corrected motor current and check if the motor is enabled asynchronously.
- **Why Needed**: It improves the responsiveness and control of the robot by leveraging asynchronous programming, which is crucial for real-time robotics applications.

### PhotonCRServo

- **Purpose**: Extends the `CRServoImplEx` class to provide asynchronous control over continuous rotation servos.
- **Key Features**:
  - **Async Direction and Power**: Methods to get the servo's direction and power asynchronously.
- **Why Needed**: Asynchronous control allows for smoother and more responsive servo operations, which is essential in robotics for precise movements.

### PhotonServo

- **Purpose**: Provides asynchronous control for standard servos.
- **Key Features**:
  - **Async Position**: Methods to check if PWM is enabled and get the servo position asynchronously.
- **Why Needed**: Enhances servo control by allowing operations to be performed without blocking the main thread, which is vital for maintaining performance in complex robotics tasks.

### PhotonLynxServoController

- **Purpose**: Manages servos connected to a Lynx module, providing asynchronous operations for servo control.
- **Key Features**:
  - **PWM Control**: Asynchronously checks if PWM is enabled and retrieves servo positions.
- **Why Needed**: Provides non-blocking operations that help in managing multiple servos efficiently in a robotics application.

### PhotonAdvancedDcMotor

- **Purpose**: Provides advanced control features for DC motors.
- **Key Features**:
  - **Cache Tolerance**: Allows setting a tolerance level for motor power changes to avoid unnecessary updates.
  - **Refresh Rate Control**: Manages how frequently motor power updates occur to optimize performance.
- **Why Needed**: Offers fine-grained control over motor operations, which is crucial for optimizing power usage and performance in competitive robotics.

### PhotonLynxCommandListener

- **Purpose**: Interface for listening to Lynx commands and their responses.
- **Key Features**:
  - **Command Handling**: Methods to handle commands and responses from the Lynx module.
- **Why Needed**: Essential for monitoring and reacting to command execution in real-time, ensuring that the robot's operations are synchronized and efficient.

---

## Installation Guide

To install PhotonCore for your FTC robotics project, follow these steps:

1. **Download the Library**: Clone or download the PhotonCore repository from GitHub.
   
2. **Integrate with FTC SDK**:
   - Copy the PhotonCore directory into your FTC project’s `src/main/java` directory.

3. **Configure Build Scripts**:
   - Ensure your `build.gradle` file includes the necessary dependencies for PhotonCore.

4. **Sync Project**: Open your project in Android Studio and sync to ensure all dependencies are correctly configured.

---

## Usage Guide

To use PhotonCore in your FTC robotics project:

1. **Initialize Components**:
   - Use the classes provided by PhotonCore to initialize motors and servos in your robot's hardware map.

2. **Implement Asynchronous Operations**:
   - Replace standard synchronous calls with PhotonCore's asynchronous methods to enhance performance.

3. **Control Motors and Servos**:
   - Utilize advanced features like cache tolerance and refresh rate control for precise motor and servo operations.

4. **Monitor Commands**:
   - Implement the `PhotonLynxCommandListener` to handle and respond to Lynx commands effectively.

---

## Conclusion

PhotonCore enhances the FTC SDK by providing advanced control features and asynchronous operations for motors and servos. By integrating PhotonCore, teams can achieve smoother and more efficient robot control, which is crucial for competitive robotics.

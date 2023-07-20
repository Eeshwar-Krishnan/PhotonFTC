package com.outoftheboxrobotics.photoncore.HAL.Motors;

import com.outoftheboxrobotics.photoncore.HAL.HAL;

import java.util.HashMap;

public class PhotonLynxDCMotorController {
    private HAL hal;
    private HashMap<Integer, PhotonDcMotor> motors;

    public PhotonLynxDCMotorController(HAL hal){
        this.hal = hal;
        motors = new HashMap<>();
    }

    public PhotonDcMotor getMotor(int port){
        if(!motors.containsKey(port)){
            motors.put(port, new PhotonDcMotor(hal, port));
        }
        return motors.get(port);
    }
}

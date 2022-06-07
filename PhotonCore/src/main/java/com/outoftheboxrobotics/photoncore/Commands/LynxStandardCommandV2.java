package com.outoftheboxrobotics.photoncore.Commands;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.commands.LynxMessage;
import com.qualcomm.hardware.lynx.commands.standard.LynxAck;
import com.qualcomm.hardware.lynx.commands.standard.LynxNack;
import com.qualcomm.hardware.lynx.commands.standard.LynxStandardCommand;

public abstract class LynxStandardCommandV2<RESPONSE extends LynxMessage> extends LynxStandardCommand<RESPONSE> {
    private boolean responded = false;
    private boolean acked = false, nacked = false;

    private ResponseCallback callback = null;

    private RESPONSE response;

    public LynxStandardCommandV2(LynxModule module) {
        super(module);
    }

    @Override
    public void onAckReceived(LynxAck ack) {
        acked = true;
        responded = true;
        if(ack.isAttentionRequired()){
            module.noteAttentionRequired();
        }
    }

    @Override
    public void onNackReceived(LynxNack nack) {
        nacked = true;
        responded = true;
    }

    @Override
    public void onResponseReceived(LynxMessage response) {
        this.response = (RESPONSE) response;
        if(callback != null){
            callback.call(response);
        }
        this.responded = true;
    }

    public void registerCallback(ResponseCallback callback){
        this.callback = callback;
    }

    public void reset(){
        this.responded = false;
        this.acked = false;
        this.nacked = false;
        this.response = null;
    }

    public boolean isResponded() {
        return responded;
    }

    public boolean isAcked() {
        return acked;
    }

    public boolean isNacked() {
        return nacked;
    }

    public RESPONSE getResponse() {
        return response;
    }
}

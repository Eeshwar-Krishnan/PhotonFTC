package com.outoftheboxrobotics.photoncore;

import java.util.concurrent.CompletableFuture;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicLong;
import java.util.concurrent.atomic.AtomicReference;

public class PeriodicSupplier<T> {
    private final AtomicReference<T> value;
    private long period;
    protected final AtomicLong timer;
    protected final AtomicBoolean inFlight;
    private final FutureSupplier supplier;

    public PeriodicSupplier(FutureSupplier<T> supplier, long period){
        this.period = period;
        timer = new AtomicLong(System.currentTimeMillis() + period);
        inFlight = new AtomicBoolean(false);
        value = new AtomicReference<>();
        this.supplier = supplier;
    }

    public void update(){
        if(!inFlight.get() && System.currentTimeMillis() > timer.get()){
            CompletableFuture<T> future = supplier.getSupplier();
            inFlight.set(true);
            future.thenAccept((result) -> {
                this.value.set(result);
                inFlight.set(false);
            });
        }
    }

    public T get(){
        return value.get();
    }

    public interface FutureSupplier<T> {
        CompletableFuture<T> getSupplier();
    }
}

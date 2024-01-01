package com.outoftheboxrobotics.photoncore;

import java.lang.annotation.Documented;
import java.lang.annotation.ElementType;
import java.lang.annotation.Inherited;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;
/**
 * Add this to a {@link com.qualcomm.robotcore.eventloop.opmode.OpMode} in order to apply PhotonCore optimizations
 */
@Documented
@Inherited
@Target(ElementType.TYPE)
@Retention(RetentionPolicy.RUNTIME)
public @interface Photon {
    int maximumParallelCommands() default 8;

    boolean singleThreadOptimized() default true;
}

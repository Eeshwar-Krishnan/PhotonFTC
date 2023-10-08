package com.outoftheboxrobotics.photoncore;

import com.outoftheboxrobotics.photoncore.hardware.servo.PhotonServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.LastKnown;

public class PhotonLastKnown<T> {
    //----------------------------------------------------------------------------------------------
    // State
    //----------------------------------------------------------------------------------------------

    protected T           value;
    protected boolean     isValid;
    protected ElapsedTime timer;
    protected double      msFreshness;
    protected boolean canExpire;

    //----------------------------------------------------------------------------------------------
    // Construction
    //----------------------------------------------------------------------------------------------

    public PhotonLastKnown()
    {
        this(500, true);
    }
    public PhotonLastKnown(boolean canExpire)
    {
        this(0,canExpire);
    }
    public PhotonLastKnown(double msFreshness, boolean canExpire)
    {
        this.value       = null;
        this.isValid     = false;
        this.timer       = new ElapsedTime();
        this.msFreshness = msFreshness;
        this.canExpire   = canExpire;
    }

    public static <X> LastKnown<X>[] createArray(int length)
    {
        LastKnown<X>[] result = new LastKnown[length];
        for (int i = 0; i < length; i++)
            result[i] = new LastKnown<X>();
        return result;
    }

    public static <X> void invalidateArray(LastKnown<X>[] array)
    {
        for (int i = 0; i < array.length; i++)
        {
            array[i].invalidate();
        }
    }

    //----------------------------------------------------------------------------------------------
    // Operations
    //----------------------------------------------------------------------------------------------

    /**
     * Marks the last known value as invalid. However, the value internally stored is uneffected.
     * @see #getRawValue()
     * @see #getValue()
     */
    public void invalidate()
    {
        this.isValid = false;
    }

    /**
     * Returns whether a last value is currently known and is fresh enough. Note that a value
     * which is valid may spontaneously become invalid (because it expires) but a value which
     * is invalid will never spontaneously become valid.
     * @return whether a last value is currently known and is fresh enough
     */
    public boolean isValid()
    {
        return this.isValid && (!canExpire || this.timer.milliseconds() <= msFreshness);
    }

    /**
     * Returns the last known value, or null if not valid
     * @return the last known value
     */
    public T getValue()
    {
        return this.isValid() ? this.value : null;
    }

    /**
     * Returns the stored value, w/o using a timer to invalidate
     * @return the raw stored value.
     */
    public T getNonTimedValue()
    {
        return this.isValid ? this.value : null;
    }

    /**
     * Returns the stored value, whether or not it is valid
     * @return the raw stored value.
     */
    public T getRawValue()
    {
        return this.value;
    }

    /**
     * If non-null, sets the current value to be the indicated (known) value and resets
     * the freshness timer. If null, this is equivalent to {@link #invalidate()}.
     * @return the previous value.
     */
    public T setValue(T value)
    {
        T prevValue = this.value;
        this.value = value;
        this.isValid = true;
        if (null == value)
            invalidate();
        else
            this.timer.reset();
        return prevValue;
    }

    /**
     * Answers whether the last known value is both valid and equal to the value indicated. Note
     * that the .equals() method is used to make the comparison.
     * @param valueQ the value queried
     * @return whether the last known value is both valid and equal to the value indicated
     */
    public boolean isValue(T valueQ)
    {
        if (this.isValid())
            return this.value.equals(valueQ);
        else
            return false;
    }

    /**
     * If the last known value is not both valid and equal to the indicated value, updates it to be
     * same and returns true; otherwise, returns false.
     * @param valueQ the value queried
     * @return whether the value was just updated
     */
    public boolean updateValue(T valueQ)
    {
        if (!isValue(valueQ))
        {
            setValue(valueQ);
            return true;
        }
        else
            return false;
    }

}


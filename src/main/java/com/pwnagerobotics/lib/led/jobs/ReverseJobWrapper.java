package com.pwnagerobotics.lib.led.jobs;

import java.util.ArrayList;
import java.util.Collections;

import edu.wpi.first.wpilibj.util.Color;


/**
 * flips the buffer of the wrapped job so its reversed
 */
public class ReverseJobWrapper extends JobWrapper {

    /**
     * @see JobWrapper#JobWrapper(Job job)
     */
    public ReverseJobWrapper(Job job) {
        super(job);
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void exec(double timestamp, ArrayList<Color> buffer) {
        super.exec(timestamp, buffer);
        Collections.reverse(buffer);
    }
}

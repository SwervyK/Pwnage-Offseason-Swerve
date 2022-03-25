package com.pwnagerobotics.lib.led.jobs;

import edu.wpi.first.wpilibj.util.Color;
import java.util.ArrayList;


/**
 * wrapps a Job so that some action can be preformed around its {@link Job#exec(double timestamp, ArrayList<Color> buffer)} method
 */
public abstract class JobWrapper implements Job {
    private Job wrappedJob = null;

    /**
     * @param job the wrapped job
     */
    public JobWrapper(Job job) {
        wrappedJob = job;
    }

    /**
     * {@inheritDoc}
     */
    public void exec(double timestamp, ArrayList<Color> buffer) { wrappedJob.exec(timestamp, buffer); }

    /**
     * {@inheritDoc}
     */
    public int getIterationCount() { return wrappedJob.getIterationCount(); }

    /**
     * {@inheritDoc}
     */
    public void expire(ArrayList<Color> buffer) { wrappedJob.expire(buffer); }

    /**
     * {@inheritDoc}
     */
    public boolean isExpired() { return wrappedJob.isExpired(); }

    /**
     * {@inheritDoc}
     */
    public void reset() { wrappedJob.reset(); }
}

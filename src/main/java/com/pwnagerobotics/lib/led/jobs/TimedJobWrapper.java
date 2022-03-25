package com.pwnagerobotics.lib.led.jobs;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.util.Color;


/**
 * expires the job after a certain amount of iterations
 */
public class TimedJobWrapper extends JobWrapper {
    private double expirationTime = 0.0;
    private double timestamp = 0.0;

    /**
     * @param expirationTime the amount in seconds until the job expires
     * @see #TimedJobWrapper(Job job, double expirationTime)
     */
    public TimedJobWrapper(Job job, double timestamp, double expirationTime) {
        super(job); 
        this.expirationTime = expirationTime;
        this.timestamp = timestamp;
    }

    /**
     * @param timestamp the current time
     * @see JobWrapper#JobWrapper(Job job)
     */
    public TimedJobWrapper(Job job, double expirationTime) {
        this(job, -1.d, expirationTime);
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void exec(double timestamp, ArrayList<Color> buffer) {
        if (this.timestamp < 0) {
            this.timestamp = timestamp;
        }
        if (timestamp > expirationTime + this.timestamp && !isExpired())
            expire(buffer);
        else
            super.exec(timestamp, buffer);
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void reset() {
        super.reset();
        timestamp = -1.d;
    }
}

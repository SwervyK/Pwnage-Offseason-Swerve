package com.pwnagerobotics.lib.led.jobs;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.util.Color;

/**
 * expires the job after a certain amount of iterations
 */
public class IterativeJobWrapper extends JobWrapper {
    private int iterationCount;

    /**
     * @param iterationCount the number of iterations that the job will have
     * @see JobWrapper#JobWrapper(Job job)
     * @see JobBasic#getIterationCount()
     */
    public IterativeJobWrapper(Job job, int iterationCount) {
        super(job);
        this.iterationCount = iterationCount;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void exec(double timestamp, ArrayList<Color> buffer) {
        if (getIterationCount() > iterationCount && !isExpired())
            expire(buffer);
        else
            super.exec(timestamp, buffer);
    }
    
}

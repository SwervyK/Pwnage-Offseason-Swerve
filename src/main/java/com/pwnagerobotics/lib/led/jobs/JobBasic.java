package com.pwnagerobotics.lib.led.jobs;

import edu.wpi.first.wpilibj.util.Color;
import java.util.ArrayList;


public abstract class JobBasic implements Job {
    protected Color[] colors; 
    protected int iterations = 0;
    private boolean isExpired = false; 

    /**
     * @param colorarray The array of colors the job will cycle between
     */
    public JobBasic(Color[] colorarray) {
        colors = colorarray;
    }

    /**
     * {@inheritDoc}
     */
    public abstract void exec(double timestamp, ArrayList<Color> buffer);

    /**
     * {@inheritDoc}
     */
    public int getIterationCount() { return iterations; }

    /**
     * {@inheritDoc}
     */
    public void expire(ArrayList<Color> buffer)
    {
        if(buffer != null) {
            // boolean allLEDSet = false;
            for (int i = 0; i < buffer.size(); i++) {
                buffer.set(i, Color.kBlack);
            }
        }
        isExpired = true;
    }

    /**
     * {@inheritDoc}
     */
    public boolean isExpired() { return isExpired; }

    /**
     * {@inheritDoc}
     */
    public void reset() { isExpired = false; iterations = 0; }
}

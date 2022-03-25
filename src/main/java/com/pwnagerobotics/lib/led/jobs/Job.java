package com.pwnagerobotics.lib.led.jobs;

import edu.wpi.first.wpilibj.util.Color;
import java.util.ArrayList;

public interface Job {
    /**
     * set data to the buffer
     * @param timestamp the current time
     * @param buffer the buffer to act upon
     */
    public void exec(double timestamp, ArrayList<Color> buffer);

    /**
     * gets the number of times the job has performed an action
     * @return number of actions preformed
     */
    public int getIterationCount();

    /**
     * sets the job as complete and sets the leds under its control to black
     * @param buffer the buffer to act upon
     */
    public void expire(ArrayList<Color> buffer);

    /**
     * gets whether the job is e
     * @return job expiration status
     */
    public boolean isExpired();

    /**
     * resets the job to its the default state as if it was just created
     */
    public void reset();
}

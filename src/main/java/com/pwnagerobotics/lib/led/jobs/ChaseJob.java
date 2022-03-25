package com.pwnagerobotics.lib.led.jobs;

import edu.wpi.first.wpilibj.util.Color;

import java.util.ArrayList;


/**
 * SolidJob but the pattern shifts to the right based on the time
 * Color array:              { Color.kBlue, Color.kRed, Color.kGreen }
 * resulting led pattern1:    { b, r, g, b, r, g, b, r, g }
 * resulting led pattern2:    { g, b, r, g, b, r, g, b, r }
 * resulting led pattern3:    { r, g, b, r, g, b, r, g, b }
 */
public class ChaseJob extends JobBasic {
    private double nextFlash = 0;
    private double interval = 0;

    /**
     * @param delay The delay, in seconds. 
     * @see JobBasic#JobBasic(Color[] colorarray)
     */
    public ChaseJob(Color[] colorarray, double delay) {
        super(colorarray);
        interval = delay;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void exec(double timestamp, ArrayList<Color> buffer) {
        for (int i = 0; i < buffer.size(); i++) {
            buffer.set(i, colors[ (i + colors.length - iterations%colors.length)%colors.length ]);
        }

        if (timestamp >= nextFlash) {
            iterations++;
            nextFlash = timestamp + interval;
        }
    }
}

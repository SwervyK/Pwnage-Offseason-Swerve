package com.pwnagerobotics.lib.led.jobs;

import edu.wpi.first.wpilibj.util.Color;

import java.util.ArrayList;


/**
 * Flashes through colors
 * Color array:              { Color.kBlue, Color.kRed, Color.kGreen }
 * resulting led pattern1:   { b, b, b, b, b, b, b, b, b }
 * resulting led pattern2:   { r, r, r, r, r, r, r, r, r }
 * resulting led pattern3:   { g, g, g, g, g, g, g, g, g }
 */
public class FlashJob extends JobBasic {
    private double nextFlash = 0.0;
    private double interval = 0.0;

    /**
     * @param delay The delay, in seconds. 
     * @see JobBasic#JobBasic(Color[] colorarray)
     */
    public FlashJob(Color[] colorarray, double delay) {
        super(colorarray);
        interval = delay;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void exec(double timestamp, ArrayList<Color> buffer) {
        for (int i = 0; i < buffer.size(); i++) {
            buffer.set(i, colors[ iterations%colors.length ]);
        }

        if (timestamp >= nextFlash) {
            iterations++;
            nextFlash = timestamp + interval;
        }
    }
}

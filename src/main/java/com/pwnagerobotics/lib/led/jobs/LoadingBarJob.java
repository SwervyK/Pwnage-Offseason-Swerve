package com.pwnagerobotics.lib.led.jobs;

import edu.wpi.first.wpilibj.util.Color;

import java.util.ArrayList;


/**
 * Fills up a bar as if it was loading
 * Color array:              { Color.kBlue, Color.kRed, Color.kGreen }
 * resulting led pattern1:   { b, b, b, b, b, b, b, b, b }
 * resulting led pattern2:   { r, b, b, b, b, b, b, b, b }
 * resulting led pattern3:   { r, r, b, b, b, b, b, b, b }
 * resulting led pattern10:  { r, r, r, r, r, r, r, r, r }
 * resulting led pattern11:  { g, r, r, r, r, r, r, r, r }
 * resulting led pattern12:  { g, g, r, r, r, r, r, r, r }
 */
public class LoadingBarJob extends JobBasic {
    private double nextFlash = 0.0;
    private double interval = 0.0;
    /**
     * @param delay The delay, in seconds. 
     * @see JobBasic#JobBasic(Color[] colorarray)
     */
    public LoadingBarJob(Color[] colorarray, double delay) {
        super(colorarray);
        interval = delay;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void exec(double timestamp, ArrayList<Color> buffer) {
        Color backgroundColor = colors[ iterations/buffer.size()%colors.length ];
        Color foregroundColor = colors[ (iterations/buffer.size() + 1)%colors.length ];

        for (int i = 0; i < buffer.size(); i++) {
            if (i < iterations%buffer.size()) {
                buffer.set(i, foregroundColor);
            } else {
                buffer.set(i, backgroundColor);
            }
        }

        if (timestamp > nextFlash) {
            iterations++;
            nextFlash = timestamp + interval;
        }
    }
}

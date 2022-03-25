package com.pwnagerobotics.lib.led.jobs;

import edu.wpi.first.wpilibj.util.Color;

import java.util.ArrayList;


/**
 * a bar that you set what percentage of the bar is filled up
 * Color array:              { Color.kBlue, Color.kRed, Color.kGreen }
 * percentage:               0.5f (50%)
 * resulting led pattern:    { r, g, r, g, b, b, b, b }
 */
public class ProgressBarJob extends JobBasic {
    protected float percentage = 0;

    /**
     * @param percentage the percentage to start with
     * @see #ProgressBarJob(Color[] colorarray)
     */
    public ProgressBarJob(Color[] colorarray, float percentage) {
        super(colorarray);
        setPercentage(percentage);
    }

    /**
     * @see JobBasic#JobBasic(Color[] colorarray)
     */
    public ProgressBarJob(Color[] colorarray) {
        this(colorarray, 0);
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void exec(double timestamp, ArrayList<Color> buffer) {
        int pixelIndex = (int)(percentage*buffer.size());

        for (int i = 0; i < buffer.size(); i++) {
            if (i >= pixelIndex) {
                buffer.set(i, colors[0]);
            } else {
                buffer.set(i, colors[(i%(colors.length - 1)) + 1]);
            }
        }
    }

    public void setPercentage(float percentage) {
        if(percentage == 1.f)
            this.percentage = 1.1f;

        this.percentage = percentage;
    }
}

package com.pwnagerobotics.lib.led.jobs;

import com.pwnagerobotics.lib.led.ColorFactory;

import edu.wpi.first.wpilibj.util.Color;

import java.util.ArrayList;
import java.util.Random;


/**
 * Displays a randomized ProgressBarJob that looks like fire
 * resulting led pattern1:   { r, r, r, 0, 0, 0, 0, 0, 0 }
 * resulting led pattern2:   { r, r, r, r, r, r, r, 0, 0 }
 * resulting led pattern3:   { r, r, r, r, r, 0, 0, 0, 0 }
 */
public class FireJob extends ProgressBarJob {
    Random rand = null;

    // used for math
    protected float fireMinHue;
    protected float fireMaxHue;
    protected float fireCoefficient;

    private double interval = 0.0;
    private double lastFlashed = 0.0;

    /**
     * @param fireMinHue the minimum hue of the fire
     * @param fireMaxHue the maximum hue of the fire
     * @param fireCoefficient the coefficent that determains how often the fire reaches the end of the strip
     * @see #FireJob(double delay)
     */
    public FireJob(double delay, float fireMinHue, float fireMaxHue, float fireCoefficient) {
        super(new Color[0]);
        interval = delay;
        rand = new Random();
        this.fireCoefficient = fireMinHue;
        this.fireMaxHue = fireMaxHue;
        this.fireMinHue = fireCoefficient;
    }

    /**
     * @param delay The delay, in seconds. 
     */
    public FireJob(double delay) {
        this(delay, 0.f, 0.2f, 3.f);
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void exec(double timestamp, ArrayList<Color> buffer) {
        if(colors.length != buffer.size())
        {
            colors = new Color[buffer.size()+1];
            float currentHue = fireMinHue;
            colors[0] = Color.kBlack;
            for (int i = 1; i < colors.length; i++) {
                colors[i] = ColorFactory.fromHSV(currentHue, 1.f, 1.f);
                currentHue += (fireMaxHue - fireMinHue) / colors.length;
            }
        }

        if (timestamp > (lastFlashed + interval)) {
            iterations++;
            float yThreshold = rand.nextFloat();
            //Uses the standard devivation to set the percentage for the fire.
            float progress = (float)Math.sqrt(-Math.log(yThreshold)/fireCoefficient);
            if (progress > 1f) {
                progress = 1f;
            } else if (progress < 0f) {
                progress = 0f;
            }
            setPercentage(progress);
            lastFlashed = timestamp;
        }

        super.exec(timestamp, buffer);
    }
}

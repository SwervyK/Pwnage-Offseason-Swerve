package com.pwnagerobotics.lib.led.jobs;

import com.pwnagerobotics.lib.led.ColorFactory;

import edu.wpi.first.wpilibj.util.Color;



/**
 * is a ChaseJob but the colors contain a rainbow at the desired pixel width
 */
public class RainbowJob extends ChaseJob {
    /**
     * @param resolution the length of the rainbow in pixels
     * @see #RainbowJob(double delay)
     */
    public RainbowJob(int resolution, double delay) {
        super(new Color[0], delay);
        colors = new Color[resolution];
        float currentHue = 0;
        for (int i = 0; i < colors.length; i++) {
            colors[i] = ColorFactory.fromHSV(currentHue, 1.f, 1.f);
            currentHue += 1.f / colors.length;
        }
    }

    /**
     * @see ChaseJob#ChaseJob(Color[] colorarray, double delay)
     */
    public RainbowJob(double delay) {
        this(10, delay);
    }
}

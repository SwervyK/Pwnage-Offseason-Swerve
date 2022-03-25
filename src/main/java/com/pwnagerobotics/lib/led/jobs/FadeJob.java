package com.pwnagerobotics.lib.led.jobs;

import edu.wpi.first.wpilibj.util.Color;

import java.util.ArrayList;

import com.pwnagerobotics.lib.led.ColorFactory;


/**
 * fades through a cycle of colors
 * Color array:              { Color.kBlue, Color.kRed, Color.kGreen }
 * resulting led pattern1:    { b, b, b, b, b, b, b, b, b }
 * resulting led pattern2:    { br, br, br, br, br, br, br, br, br }
 * resulting led pattern3:    { r, r, r, r, r, r, r, r, r }
 */
public class FadeJob extends JobBasic {
    protected double startTime;
    protected double fadeSeconds;

    /**
     * @param fadeSeconds the amount of time to fully switch from one color to another
     * @see JobBasic#JobBasic(Color[] colorarray)
     */
    public FadeJob(Color[] colorarray, double fadeSeconds) {
        super(colorarray);
        startTime = -1;
        this.fadeSeconds = fadeSeconds;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void exec(double timestamp, ArrayList<Color> buffer) {
        if(startTime == -1) 
            startTime = timestamp;
        iterations = (int)((timestamp - startTime)/fadeSeconds);

        Color color = fade(colors[iterations%colors.length], colors[(iterations+1)%colors.length], ((timestamp - startTime)%fadeSeconds)/fadeSeconds);

        for (int i = 0; i < buffer.size(); i++) {
            buffer.set(i, color);
        }
    }

    private Color fade(Color c1, Color c2, double alpha) {
        double r = ((1 - alpha) * c1.red   + alpha * c2.red);
        double g = ((1 - alpha) * c1.green + alpha * c2.green);
        double b = ((1 - alpha) * c1.blue  + alpha * c2.blue);
        return ColorFactory.fromRGB(r, g, b);
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void reset() { startTime = -1; super.reset(); }
}

package com.pwnagerobotics.lib.led.jobs;

import com.pwnagerobotics.lib.led.ColorFactory;
import edu.wpi.first.wpilibj.util.Color;
import java.util.ArrayList;



/**
 * Fades (like someone breathing) between a color and black.
 * See https://sean.voisen.org/blog/2011/10/breathing-led-with-arduino
 */
public class BreatheJob extends JobBasic {
    protected double mStartTime;
    protected double mFadeCoefficient;

    public BreatheJob(Color[] colorarray, double fadeCoefficient) {
        super(colorarray);
        mStartTime = -1;
        mFadeCoefficient = fadeCoefficient;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void exec(double timestamp, ArrayList<Color> buffer) {
        if(mStartTime == -1) 
            mStartTime = timestamp;

        Color color = breathe(colors[0], timestamp);

        for (int i = 0; i < buffer.size(); i++) {
            buffer.set(i, color);
        }
    }

    private Color breathe(Color c1, double timestamp) {
        double alpha = (Math.exp(Math.sin(timestamp*mFadeCoefficient)) - 0.36787944)*0.42352941;
        //System.out.println(alpha);
        double r = (alpha * c1.red);
        double g = (alpha * c1.green);
        double b = (alpha* c1.blue);
        // double r = ((1-alpha) * c1.red);
        // double g = ((1-alpha) * c1.green);
        // double b = ((1-alpha) * c1.blue);        
        return ColorFactory.fromRGB(r, g, b);
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void reset() { mStartTime = -1; super.reset(); }
}

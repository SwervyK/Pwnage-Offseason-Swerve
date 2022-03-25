package com.pwnagerobotics.lib.led.jobs;

import edu.wpi.first.wpilibj.util.Color;

import java.util.ArrayList;


/**
 * displays unanimated colors
 * it cycles through an array you provide each led being the next color
 * Color array:              { Color.kBlue, Color.kRed, Color.kGreen }
 * resulting led pattern:    { b, r, g, b, r, g, b, r, g }
 */
public class SolidJob extends JobBasic {
    /**
     * @see JobBasic#JobBasic(Color[] colorarray)
     */
    public SolidJob(Color[] colorarray) {
        super(colorarray);
    }

    /**
     * @see JobBasic#JobBasic(Color[] colorarray)
     */
    public SolidJob(Color color) {
        this(new Color[] {color});
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void exec(double timestamp, ArrayList<Color> buffer) {
        for (int i = 0; i < buffer.size(); i++) {
            buffer.set(i, colors[ i%colors.length ]);
        }
        iterations++;
    }
}

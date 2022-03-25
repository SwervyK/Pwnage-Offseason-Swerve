package com.pwnagerobotics.lib.led.jobs;

import edu.wpi.first.wpilibj.util.Color;

import java.util.ArrayList;


/**
 * BinaryJob displays an integer as binary using one color for 0 and other for 1
 * Color array:              { Color.kBlue, Color.kRed }
 * input integer:            7
 * resulting led pattern:    { r, r, r, b, b, b, b, b, b }
 */
public class BinaryJob extends SolidJob {
    private final Color[] colorRefrence;

    /**
     * @param binary the initial byte of binary data to set the strip to
     * @see #BinaryJob(Color[] colorRefrence)
     */
    public BinaryJob(Color[] colorRefrence, int binary) {
        super((Color)null);
        this.colorRefrence = colorRefrence;
        setBinary(binary);
    }

    /**
     * @param colorRefrence first Color is used for binary value 1 second Color is used for 0
     */
    public BinaryJob(Color[] colorRefrence) {
        this(colorRefrence, 0b0);
    }

    /**
     * sets the binary data to display
     * @param binary the integer to create the binary array for
     */
    public void setBinary(int binary) {
        ArrayList<Color> colorArray = new ArrayList<>();

        for(int c = 0; c < Integer.SIZE; c++) {
            colorArray.add(((binary & 0b1) == 0b1)? colorRefrence[0] : colorRefrence[1]);
            binary >>= 1;
        }

        colors = colorArray.toArray(new Color[0]);
    }
}

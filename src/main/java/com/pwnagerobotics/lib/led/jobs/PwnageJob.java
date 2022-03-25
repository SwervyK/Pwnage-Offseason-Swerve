package com.pwnagerobotics.lib.led.jobs;

import edu.wpi.first.wpilibj.util.Color;
import java.util.ArrayList;

public class PwnageJob extends JobBasic {
    
    // ** Config ** //
    private float bChange = 0.01f; // how fast pixels change brightness between 0-1
    private float maxColor = 1f; // maximum brightness for a pixel between 0-1
    // ** Config ** //
    
    public PwnageJob(Color[] colorarray, double delay) {
        super(colorarray);
        interval = delay;
    }

    private float[] pixelColor;
    private boolean[] isGoingUp;
    private boolean[] active;
    private double nextFlash = 0;
    private double interval = 0;
    private double currentIterations = 0;
    
    @Override
    public void exec(double timestamp, ArrayList<Color> buffer) {
        if (pixelColor == null) {
            pixelColor = new float[buffer.size()];
            isGoingUp = new boolean[buffer.size()];
            active = new boolean[buffer.size()];
        }

        if (timestamp >= nextFlash) {
            iterations++;
            nextFlash = timestamp + interval;
        }

        if (currentIterations != iterations) {
            int index = (int) (Math.random() * buffer.size());
            pixelColor[index] = 0.01f;
            isGoingUp[index] = true;
            active[index] = true;
            currentIterations = iterations;
        }
        
        for (int i = 0; i < buffer.size(); i++) {
            buffer.set(i, colorAtIndex(i));
        }
    }
    
    public Color colorAtIndex(int index) {
        pixelColor[index] += (isGoingUp[index]) ? bChange : -bChange;
        if (pixelColor[index] > maxColor) {
            isGoingUp[index] = false;
            pixelColor[index] = maxColor;
        }
        if (pixelColor[index] < 0) {
            active[index] = false;
        }

        if (active[index]) {
            if (index % 2 == 0) {
                return new Color(pixelColor[index], pixelColor[index], 1);
            }
            else {
                return new Color(pixelColor[index], 1, pixelColor[index]);
            }
            //return new Color(pixelColor[index], pixelColor[index], pixelColor[index]);
        } else {
            if (index % 2 == 0) {
                return Color.kBlue;
            }
            else {
                return Color.kLime;
            }
        }
    }
}
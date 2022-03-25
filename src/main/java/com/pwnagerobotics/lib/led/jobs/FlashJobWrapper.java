package com.pwnagerobotics.lib.led.jobs;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.util.Color;

public class FlashJobWrapper extends JobWrapper {

    private double nextFlash = 0.0;
    private double interval = 0.0;
    private int iterations = 0;

    public FlashJobWrapper(Job job, double interval) {
        super(job);
        this.interval = interval;
    }

    @Override
    public void exec(double timestamp, ArrayList<Color> buffer) {
        if (iterations % 2 == 0) {
            for (int i = 0; i < buffer.size(); i++) {
                buffer.set(i, Color.kBlack);
            }
        }
        else {
            super.exec(timestamp, buffer);
        }
        
        if (timestamp >= nextFlash) {
            iterations++;
           nextFlash = timestamp + interval;
        }
    }
}

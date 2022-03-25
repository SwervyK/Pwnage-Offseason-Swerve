package com.team254.lib.auto.actions;

import edu.wpi.first.wpilibj.Timer;

/**
 * Action to wait for a given amount of time To use this Action, call runAction(new WaitAction(your_time))
 */
public class WaitAction implements Action {
    private final double mTimeToWait;
    private double mStartTime;

    public WaitAction(double seconds) {
        mTimeToWait = seconds;
    }

    @Override
    public void start() {
        mStartTime = Timer.getFPGATimestamp();
    }

    @Override
    public void update() {}

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - mStartTime >= mTimeToWait;
    }

    @Override
    public void done() {}
}

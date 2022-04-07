package com.pwnagerobotics.pwnage2022;

import com.team254.lib.vision.GoalTrack.GoalTrackConstants;

/**
 * A list of constants used by the rest of the robot code. This includes physics
 * constants as well as constants determined through calibration.
 */
public class Constants {

    // Swerve

    // Drive Wheels
    public static final double kDriveWheelTrackWidthInches = 30.5; //28.25;
    public static final double kTrackScrubFactor = 1.0;
    
    // GoalTrack
    public static final GoalTrackConstants kGoalTrackConstants = new GoalTrackConstants();
    static {
        kGoalTrackConstants.kMaxTrackerDistance = 20.0;
        kGoalTrackConstants.kMaxGoalTrackAge = 2.5;
        kGoalTrackConstants.kMaxGoalTrackSmoothingTime = 0.5;
        kGoalTrackConstants.kCameraFrameRate = 90.0;
    }
}
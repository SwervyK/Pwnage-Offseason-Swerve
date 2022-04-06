package com.pwnagerobotics.pwnage2022;

/**
 * A list of constants used by the rest of the robot code. This includes physics
 * constants as well as constants determined through calibration.
 */
public class Constants {

    // Drive Wheels
    public static final double kdriveWheelMultiplier = 0.0;
    public static final double kDriveWheelTrackWidthInches = 30.5; //28.25;
    public static final double kTrackScrubFactor = 1.0;
    
    public static final double kMaxTrackerDistance = 20.0;
    public static final double kMaxGoalTrackAge = 2.5;
    public static final double kMaxGoalTrackSmoothingTime = 0.5;
    public static final double kImageCaptureLatency = 11.0 / 1000.0; // seconds
    public static final double kCameraFrameRate = 90.0; 
}
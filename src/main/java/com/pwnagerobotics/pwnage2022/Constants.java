package com.pwnagerobotics.pwnage2022;

/**
 * A list of constants used by the rest of the robot code. This includes physics
 * constants as well as constants determined through calibration.
 */
public class Constants {

    // private static final String kPracticeMacAddress = "00-80-2F-33-D1-71";
    // public static final IRobotProfile kRobotProfile = RobotProfile.getRobotProfile(kPracticeMacAddress);
    
    // Drive Wheels
    public static final double kdriveWheelMultiplier = 0.0;
    public static final double kDriveWheelTrackWidthInches = 30.5; //28.25;
    // public static final double kDriveWheelDiameterInches = 3.875 * kRobotProfile.driveWheelMultiplier();// practice 0.960125;
    // public static final double kDriveWheelRadiusInches = kDriveWheelDiameterInches / 2.0;
    // public static final double kDriveWheelTrackRadiusWidthMeters = kDriveWheelTrackWidthInches / 2.0 * 0.0254;
    public static final double kTrackScrubFactor = 1.0;


    
    public static final double kMaxTrackerDistance = 20.0;
    public static final double kMaxGoalTrackAge = 2.5;
    public static final double kMaxGoalTrackSmoothingTime = 0.5;
    public static final double kImageCaptureLatency = 11.0 / 1000.0; // seconds
    public static final double kCameraFrameRate = 90.0; 
    // Log file location
    public static final String kBaseDirectory = "/home/lvuser/";
    public static final String kLogDirectory = kBaseDirectory + "logs/";
}
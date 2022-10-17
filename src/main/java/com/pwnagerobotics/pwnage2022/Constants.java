package com.pwnagerobotics.pwnage2022;

import com.pwnagerobotics.pwnage2022.lib.SwerveModule.SwerveModuleConstants;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.vision.GoalTrack.GoalTrackConstants;

/**
 * A list of constants used by the rest of the robot code. This includes physics
 * constants as well as constants determined through calibration.
 */
public class Constants {

    //
    public static final Translation2d kVehicleToTurretTranslation = new Translation2d(-0.75, 0);

    // Power Limits
    public static final double kDriveVoltageOpenLoopCompSaturation = 12.0;
    public static final int kDriveCurrentLimit = 60;

    // Goal Trackers
    public static final double kDefaultMaxTrackAge = 2.5;
    public static final double kMaxTrackerDistance = 20.0;
    public static final double kMaxGoalTrackAge = 2.5; 
    public static final double kMaxGoalTrackAgeNotTracking = 0.1;
    public static final double kMaxGoalTrackSmoothingTime = 0.5; 
    public static final double kTrackStabilityWeight = 0.0; 
    public static final double kTrackAgeWeight = 10.0;
    public static final double kTrackSwitchingWeight = 100.0;
    public static final double kCameraFrameRate = 90.0;
    public static final double kGoalHeight = 100.0;

    // Limelight
    public static final double kHorizontalFOV_1X = 59.6; // degrees
    public static final double kVerticalFOV_1X = 49.7; // degrees
    public static final double kVPW_1X = 2.0 * Math.tan(Math.toRadians(kHorizontalFOV_1X / 2.0)); 
    public static final double kVPH_1X = 2.0 * Math.tan(Math.toRadians(kVerticalFOV_1X / 2.0));
    public static final double kDistanceFudgeFactor = 1.0; // 0 to 1. decrease if overshooting
    public static final double kImageCaptureLatency = 11.0 / 1000.0;

    // GoalTrack
    public static final GoalTrackConstants kGoalTrackConstants = new GoalTrackConstants();
    static {
        kGoalTrackConstants.kMaxTrackerDistance = kMaxTrackerDistance;
        kGoalTrackConstants.kMaxGoalTrackAge = kMaxGoalTrackAge;
        kGoalTrackConstants.kMaxGoalTrackSmoothingTime = kMaxGoalTrackSmoothingTime;
        kGoalTrackConstants.kCameraFrameRate = kCameraFrameRate;
    }

    // Controller
    public static final double kDriveMaxValue = 1.0;
    public static final double kDriveMinValue = 0.08;
    public static final double kRotationMaxValue = 1.0;
    public static final double kRotationMinValue = 0.1;
    public static final double kLeftStickDeadband = 0.1;
    public static final double kRightStickDeadband = 0.15;
    public static final double kPoleSnappingAngle = 10; // +-10 deg
    public static final double kPoleSnappingThreshold = 0.5; // mag > threshold

    // Drivetrain
    public static final double kDriveWidth = 21.5; //Width between center of drive modules, inches
    public static final double kDriveLength = 21.5; //Width between center of drive modules, inches
    public static final double kDriveSlowDown = 1.0;
    public static final double kRotationSlowDown = 1.0;
    public static final double kDriveMinSpeed = 5; //Last encoder value - current encoder value
    public static final double kDriveEncoderCPR = 8192; //Encoder counts per revolution

    // Field Centric Rotation
    public static final double kRotationP = 0.008; // 0.005
    public static final double kRotationI = 0.0001; // 0.0005
    public static final double kRotationD = 0.001; // 0.0
    public static final double kFieldCentricRotationError = 2;

    // Compensation
    public static final double kCompensationP = 0.008;
    public static final double kCompensationI = 0.0001;
    public static final double kCompensationD = 0.001;
    public static final double kCompensationErrorLow = 2;
    public static final double kCompensationErrorHigh = 10;

    // Gyro
    public static final double kGyroOffset = 0.0;
    public static final double kGyroLag = 0.0; // When moving compensate for gyro lag
    public static final double kMinGyroDelta = 5; // Gyro delta to consider robot not rotating

    // Modules
    public static final SwerveModuleConstants kFrontRightModuleConstants = new SwerveModuleConstants();
    static {
        kFrontRightModuleConstants.kName = "Front Right";
        kFrontRightModuleConstants.kDriveId = 1;
        kFrontRightModuleConstants.kRotationId = 0;
        kFrontRightModuleConstants.kDriveEncoderId = new int[]{0, 1};
        kFrontRightModuleConstants.kRotationOffset =  0.092;
        kFrontRightModuleConstants.kRotationEncoderId = 2;
        kFrontRightModuleConstants.kPDPId = 3;
    }

    public static final SwerveModuleConstants kFrontLeftModuleConstants = new SwerveModuleConstants();
    static {
        kFrontLeftModuleConstants.kName = "Front Left";
        kFrontLeftModuleConstants.kDriveId = 5;
        kFrontLeftModuleConstants.kRotationId = 4;
        kFrontLeftModuleConstants.kDriveEncoderId = new int[]{4, 5};
        kFrontLeftModuleConstants.kRotationOffset =  0.126;
        kFrontLeftModuleConstants.kRotationEncoderId = 0;
        kFrontLeftModuleConstants.kPDPId = 14;
   }

    public static final SwerveModuleConstants kBackRightModuleConstants = new SwerveModuleConstants();
    static {
        kBackRightModuleConstants.kName = "Back Right";
        kBackRightModuleConstants.kDriveId = 3;
        kBackRightModuleConstants.kRotationId = 2;
        kBackRightModuleConstants.kDriveEncoderId = new int[]{2, 3};
        kBackRightModuleConstants.kRotationOffset =  0.735;
        kBackRightModuleConstants.kRotationEncoderId = 3;
        kBackRightModuleConstants.kPDPId = 0;
    }

    public static final SwerveModuleConstants kBackLeftModuleConstants = new SwerveModuleConstants();
    static {
        kBackLeftModuleConstants.kName = "Back Left";
        kBackLeftModuleConstants.kDriveId = 9;
        kBackLeftModuleConstants.kRotationId = 6;
        kBackLeftModuleConstants.kDriveEncoderId = new int[]{6, 7};
        kBackLeftModuleConstants.kRotationOffset =  0.189;
        kBackLeftModuleConstants.kRotationEncoderId = 1;
        kBackLeftModuleConstants.kPDPId = 15;
    }
}
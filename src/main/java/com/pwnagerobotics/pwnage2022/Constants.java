package com.pwnagerobotics.pwnage2022;

import com.pwnagerobotics.pwnage2022.lib.SwerveModule.SwerveModuleConstants;

/**
 * A list of constants used by the rest of the robot code. This includes physics
 * constants as well as constants determined through calibration.
 */
public class Constants {

    // Controller
    public static final double kDriveMaxValue = 1.0;
    public static final double kDriveMinValue = 0.08;
    public static final double kRotationMaxValue = 1.0;
    public static final double kRotationMinValue = 0.1;
    public static final double kLeftStickDeadband = 0.1;
    public static final double kRightStickDeadband = 0.15;
    public static final double kPoleSnappingThreshold = 10;
    
    // Drivetrain
    public static final double kDriveWidth = 21.5; //Width between center of drive modules, inches
    public static final double kDriveLength = 21.5; //Width between center of drive modules, inches
    public static final double kDriveSlowDown = 1.0;
    public static final double kRotationSlowDown = 1.0;
    public static final double kDriveCurrentLimit = 50;
    public static final double kDriveRateLimit = 1;
    public static final double kDriveMinSpeed = 5; //Last encoder value - current encoder value
    public static final double kDriveEncoderCPR = 8192; //Encoder counts per revolution

    // Field Centric Rotation
    public static final double kRotationP = 0.005;
    public static final double kRotationI = 0.0005;
    public static final double kRotationD = 0.0;
    public static final double kFieldCentricRotationError = 2;

    // Compensation
    public static final double kCompensationP = 0.005;
    public static final double kCompensationI = 0.0005;
    public static final double kCompensationD = 0.0;
    public static final double kCompensationError = 2;

    // Gyro
    public static final double kGyroOffset = 0.0;
    public static final double kGyroLag = 50.0; // When moving compensate for gyro lag
    public static final double kMinGyroDelta = 10; // Gyro delta to consider robot not rotating

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
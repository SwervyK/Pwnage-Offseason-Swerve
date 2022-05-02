package com.pwnagerobotics.pwnage2022;

import com.pwnagerobotics.pwnage2022.lib.SwerveModuleConstants;

/**
 * A list of constants used by the rest of the robot code. This includes physics
 * constants as well as constants determined through calibration.
 */
public class Constants {
    // Drivetrain
    public static final double kDriveSlowDown = 0.5;
    public static final double kRotationSlowDown = 1;
    public static final double kSpinSlowDown = 0.5;
    public static final double kDriveCurrentLimit = 50;

    // Controller Deadbands
    public static final double kLeftStickDeadband = 0.05;
    public static final double kRightStickDeadband = 0.08;

    
    // Field Centric Rotaion
    public static final double kRotationkP = 0.01;
    public static final double kRotationkI = 0.001;
    public static final double kRotationkD = 0.0;
    public static final double kFieldCentricRotationError = 2;

    // Gyro
    public static final double kGyroOffset = 284.0;
    public static final double kGyroLag = 100.0; // When moving compensate for gyro lag
    public static final boolean kTrustGyro = true;

    // Modules
    public static final SwerveModuleConstants kFrontRightModuleConstants = new SwerveModuleConstants();
    static {
        kFrontRightModuleConstants.kName = "Front Right";
        kFrontRightModuleConstants.kDriveId = 1;
        kFrontRightModuleConstants.kRotationId = 0;
        kFrontRightModuleConstants.kRotationOffset =  0.094;
        kFrontRightModuleConstants.kRotationEncoderId = 2;
    }

    public static final SwerveModuleConstants kFrontLefttModuleConstants = new SwerveModuleConstants();
    static {
        kFrontLefttModuleConstants.kName = "Front Left";
        kFrontLefttModuleConstants.kDriveId = 5;
        kFrontLefttModuleConstants.kRotationId = 4;
        kFrontLefttModuleConstants.kRotationOffset =  0.126;
        kFrontLefttModuleConstants.kRotationEncoderId = 0;
   }

    public static final SwerveModuleConstants kBackRightModuleConstants = new SwerveModuleConstants();
    static {
        kBackRightModuleConstants.kName = "Back Right";
        kBackRightModuleConstants.kDriveId = 3;
        kBackRightModuleConstants.kRotationId = 2;
        kBackRightModuleConstants.kRotationOffset =  0.735;
        kBackRightModuleConstants.kRotationEncoderId = 3;
    }

    public static final SwerveModuleConstants kBackLefttModuleConstants = new SwerveModuleConstants();
    static {
        kBackLefttModuleConstants.kName = "Back Left";
        kBackLefttModuleConstants.kDriveId = 9;
        kBackLefttModuleConstants.kRotationId = 6;
        kBackLefttModuleConstants.kRotationOffset =  0.189;
        kBackLefttModuleConstants.kRotationEncoderId = 1;
    }
}
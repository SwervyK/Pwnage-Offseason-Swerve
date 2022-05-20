package com.pwnagerobotics.pwnage2022;

import com.pwnagerobotics.pwnage2022.lib.SwerveModuleConstants;

/**
 * A list of constants used by the rest of the robot code. This includes physics
 * constants as well as constants determined through calibration.
 */
public class Constants {
    // Drivetrain
    public static final double kDriveSlowDown = 0.7;
    public static final double kRotationSlowDown = 1.0; // Field Centric Rotation
    public static final double kSpinSlowDown = 1.0; // Robot Centric Rotation
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
    public static final double kGyroLag = 50.0; // When moving compensate for gyro lag
    public static final double kGyroDelay = 1.5; // Seonds

    // Modules
    public static final SwerveModuleConstants kFrontRightModuleConstants = new SwerveModuleConstants();
    static {
        kFrontRightModuleConstants.kName = "Front Right";
        kFrontRightModuleConstants.kDriveId = 1;
        kFrontRightModuleConstants.kRotationId = 0;
        kFrontRightModuleConstants.kRotationOffset =  0.092;
        kFrontRightModuleConstants.kRotationEncoderId = 2;
        kFrontRightModuleConstants.kPDPId = 3;
    }

    public static final SwerveModuleConstants kFrontLeftModuleConstants = new SwerveModuleConstants();
    static {
        kFrontLeftModuleConstants.kName = "Front Left";
        kFrontLeftModuleConstants.kDriveId = 5;
        kFrontLeftModuleConstants.kRotationId = 4;
        kFrontLeftModuleConstants.kRotationOffset =  0.126;
        kFrontLeftModuleConstants.kRotationEncoderId = 0;
        kFrontLeftModuleConstants.kPDPId = 14;
   }

    public static final SwerveModuleConstants kBackRightModuleConstants = new SwerveModuleConstants();
    static {
        kBackRightModuleConstants.kName = "Back Right";
        kBackRightModuleConstants.kDriveId = 3;
        kBackRightModuleConstants.kRotationId = 2;
        kBackRightModuleConstants.kRotationOffset =  0.735;
        kBackRightModuleConstants.kRotationEncoderId = 3;
        kBackRightModuleConstants.kPDPId = 0;
    }

    public static final SwerveModuleConstants kBackLeftModuleConstants = new SwerveModuleConstants();
    static {
        kBackLeftModuleConstants.kName = "Back Left";
        kBackLeftModuleConstants.kDriveId = 9;
        kBackLeftModuleConstants.kRotationId = 6;
        kBackLeftModuleConstants.kRotationOffset =  0.189;
        kBackLeftModuleConstants.kRotationEncoderId = 1;
        kBackLeftModuleConstants.kPDPId = 15;
    }
}
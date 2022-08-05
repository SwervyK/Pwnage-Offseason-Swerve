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
    public static final double kDriveSlowDown = 1.0; // Module Drive
    public static final double kRotationSlowDown = 1.0; // Module Rotation
    public static final double kDriveCurrentLimit = 50;
    public static final double kDriveRateLimit = 1;
    public static final double kDriveMinSpeed = 5; //Last encoder value - current encoder value
    public static final double kDriveEncoderCPR = 8192; //Encoder counts per revolution

    // Field Centric Rotaion
    public static final double kRotationkP = 0.005;
    public static final double kRotationkI = 0.0005;
    public static final double kRotationkD = 0.0;
    public static final double kFieldCentricRotationError = 2; // 4 degrees

    // Compsentation
    public static final double kCompensationP = 0.005;
    public static final double kCompensationI = 0.0005;
    public static final double kCompensationD = 0.0;
    public static final double kCompensationError = 4; // 8 degrees
    public static final double kStartCompensation = 6; // 12 degrees

    // Gyro
    public static final double kGyroOffset = 346.75;
    public static final double kGyroLag = 50.0; // When moving compensate for gyro lag
    public static final double kGyroDelay = 2; // Seonds after turing to enable compensation 

    // Modules
    public static final SwerveModuleConstants kFrontRightModuleConstants = new SwerveModuleConstants();
    static {
        kFrontRightModuleConstants.kName = "Front Right";
        kFrontRightModuleConstants.kDriveId = 1;
        kFrontRightModuleConstants.kRotationId = 0;
        kFrontRightModuleConstants.kDriveEncoderId = 0;
        kFrontRightModuleConstants.kRotationOffset =  0.092;
        kFrontRightModuleConstants.kRotationEncoderId = 2;
        kFrontRightModuleConstants.kPDPId = 3;
    }

    public static final SwerveModuleConstants kFrontLeftModuleConstants = new SwerveModuleConstants();
    static {
        kFrontLeftModuleConstants.kName = "Front Left";
        kFrontLeftModuleConstants.kDriveId = 5;
        kFrontLeftModuleConstants.kRotationId = 4;
        kFrontLeftModuleConstants.kDriveEncoderId = 0;
        kFrontLeftModuleConstants.kRotationOffset =  0.126;
        kFrontLeftModuleConstants.kRotationEncoderId = 0;
        kFrontLeftModuleConstants.kPDPId = 14;
   }

    public static final SwerveModuleConstants kBackRightModuleConstants = new SwerveModuleConstants();
    static {
        kBackRightModuleConstants.kName = "Back Right";
        kBackRightModuleConstants.kDriveId = 3;
        kBackRightModuleConstants.kRotationId = 2;
        kBackRightModuleConstants.kDriveEncoderId = 0;
        kBackRightModuleConstants.kRotationOffset =  0.735;
        kBackRightModuleConstants.kRotationEncoderId = 3;
        kBackRightModuleConstants.kPDPId = 0;
    }

    public static final SwerveModuleConstants kBackLeftModuleConstants = new SwerveModuleConstants();
    static {
        kBackLeftModuleConstants.kName = "Back Left";
        kBackLeftModuleConstants.kDriveId = 9;
        kBackLeftModuleConstants.kRotationId = 6;
        kBackLeftModuleConstants.kDriveEncoderId = 0;
        kBackLeftModuleConstants.kRotationOffset =  0.189;
        kBackLeftModuleConstants.kRotationEncoderId = 1;
        kBackLeftModuleConstants.kPDPId = 15;
    }
}
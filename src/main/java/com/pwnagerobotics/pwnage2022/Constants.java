package com.pwnagerobotics.pwnage2022;

import com.pwnagerobotics.pwnage2022.lib.SwerveModuleConstants;

/**
 * A list of constants used by the rest of the robot code. This includes physics
 * constants as well as constants determined through calibration.
 */
public class Constants {

    // Drive
    public static final double kDriveSlowDown = 0.7;
    public static final double kTurnSlowDown = 0.7;
    public static final double kRotationSlowDown = 0.8;

    public static final double kFieldCentricRotationError = 10;

    public static final double kRotationkP = 0.7;
    public static final double kRotationkI = 0.015;
    public static final double kRotationkD = 0.0;

    public static final double kGyroOffset = 284.0;

    public static final double kDriveAccelerationLimit = 0.5;

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
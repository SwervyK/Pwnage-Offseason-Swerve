package com.pwnagerobotics.pwnage2022;

import com.pwnagerobotics.pwnage2022.lib.SwerveModuleConstants;

/**
 * A list of constants used by the rest of the robot code. This includes physics
 * constants as well as constants determined through calibration.
 */
public class Constants {

    // Drive
    public static final double kRotationDeadband = 2;
    public static final double kDriveSlowDown = 0.4;
    public static final double kRotationSlowDown = 0.4;
    public static final double kTurnSlowDown = 0.4;
    public static final double kFieldCentricRotationError = 5;
    public static final double kRotationkP = 0.7;
    public static final double kRotationkI = 0.015;
    public static final double kRotationkD = 0.0;

    public static final double kGyroOffset = 0.0;

    // Modules
    public static final SwerveModuleConstants kFrontRightModuleConstants = new SwerveModuleConstants();
    static {
        kFrontRightModuleConstants.kName = "Front Right";
        kFrontRightModuleConstants.kDriveId = 1;
        kFrontRightModuleConstants.kRotationId = 0;
        kFrontRightModuleConstants.kRotationOffset =  0.590;
        kFrontRightModuleConstants.kRotationEncoderId = 2;
        kFrontRightModuleConstants.kTurnDegree = 135;
    }

    public static final SwerveModuleConstants kFrontLefttModuleConstants = new SwerveModuleConstants();
    static {
        kFrontLefttModuleConstants.kName = "Front Left";
        kFrontLefttModuleConstants.kDriveId = 5;
        kFrontLefttModuleConstants.kRotationId = 4;
        kFrontLefttModuleConstants.kRotationOffset =  0.601;
        kFrontLefttModuleConstants.kRotationEncoderId = 0;
        kFrontLefttModuleConstants.kTurnDegree = 45;
    }

    public static final SwerveModuleConstants kBackRightModuleConstants = new SwerveModuleConstants();
    static {
        kBackRightModuleConstants.kName = "Back Right";
        kBackRightModuleConstants.kDriveId = 3;
        kBackRightModuleConstants.kRotationId = 2;
        kBackRightModuleConstants.kRotationOffset =  0.248;
        kBackRightModuleConstants.kRotationEncoderId = 3;
        kBackRightModuleConstants.kTurnDegree = 45;
    }

    public static final SwerveModuleConstants kBackLefttModuleConstants = new SwerveModuleConstants();
    static {
        kBackLefttModuleConstants.kName = "Back Left";
        kBackLefttModuleConstants.kDriveId = 9;
        kBackLefttModuleConstants.kRotationId = 6;
        kBackLefttModuleConstants.kRotationOffset =  0.185;
        kBackLefttModuleConstants.kRotationEncoderId = 1;
        kBackLefttModuleConstants.kTurnDegree = 135;
    }
}
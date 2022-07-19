package com.pwnagerobotics.pwnage2022;

import org.ejml.simple.SimpleMatrix;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.geometry.Twist2d;
import com.team254.lib.util.DriveSignal;

/**
 * Provides forward and inverse kinematics equations for the robot modeling the wheelbase as a differential drive (with
 * a corrective factor to account for skidding).
 */
// INFO means refrence
public class Kinematics {
    private static SimpleMatrix mInverseKinematics = new SimpleMatrix(4 * 2, 3); // INFO SwerveDriveKinematics.m_inverseKinematics
    static {
        // Front Right, Front Left, Rear Right, Rear Left
        double[][] modulePos = {{Constants.kDriveWidth/2, -Constants.kDriveLength/2},
        {Constants.kDriveWidth/2, Constants.kDriveLength/2},
        {-Constants.kDriveWidth/2, -Constants.kDriveLength/2},
        {-Constants.kDriveWidth/2, Constants.kDriveLength/2}};
        for (int i = 0; i < 4; i++) {
            mInverseKinematics.setRow(i * 2 + 0, 0, /* Start Data */ 1, 0, modulePos[i][1]);
            mInverseKinematics.setRow(i * 2 + 1, 0, /* Start Data */ 0, 1, modulePos[i][0]);
        }
    }
    private static SimpleMatrix mForwardKinematics = mInverseKinematics.pseudoInverse(); // INFO SwerveDriveKinematics.m_forwardKinematics

    /**
     * Forward kinematics using only encoders
     */
    // moduleStates is rotation and velocity of each module
    // velocity in Meters/Sec, rotation in degrees
    public static Pose2d forwardKinematics(Translation2d[] moduleStates) { // INFO SwerveDriveKinematics.toChassisSpeeds()
        SimpleMatrix moduleStatesMatrix = new SimpleMatrix(moduleStates.length * 2, 1);

        for (int i = 0; i < moduleStates.length; i++) {
            moduleStatesMatrix.set(i * 2, 0, Math.sqrt(moduleStates[i].norm2()) * moduleStates[i].direction().cos());
            moduleStatesMatrix.set(i * 2 + 1, Math.sqrt(moduleStates[i].norm2()) * moduleStates[i].direction().sin());
          }
      
          SimpleMatrix chassisSpeedsVector = mForwardKinematics.mult(moduleStatesMatrix);
          return new Pose2d( // INFO: ChassisSpeeds
          chassisSpeedsVector.get(0, 0), // Meters/Sec
          chassisSpeedsVector.get(1, 0), // Meters/Sec
          new Rotation2d(chassisSpeedsVector.get(2, 0), false)); // Rad/Sec
    }

    /**
     * For convenience, integrate forward kinematics with a Twist2d and previous rotation.
     */
    public static Pose2d integrateForwardKinematics(Pose2d current_pose,
                                                    Twist2d forward_kinematics) {
        return current_pose.transformBy(Pose2d.exp(forward_kinematics));
    }

    /**
     * Uses inverse kinematics to convert a Pose2d into wheel velocities
     */
    public static Translation2d[] inverseKinematics(Pose2d velocity) { // https://www.chiefdelphi.com/t/paper-4-wheel-independent-drive-independent-steering-swerve/107383
        double x = velocity.getTranslation().x();
        double y = velocity.getTranslation().y();
        double rl = velocity.getRotation().getDegrees()*Constants.kDriveLength/2;
        double rw =velocity.getRotation().getDegrees()*Constants.kDriveWidth/2;

        Translation2d frontRight = new Translation2d(x + rl, y + rw);
        Translation2d frontLeft = new Translation2d(x + rl, y - rw);
        Translation2d backRight = new Translation2d(x - rl, y + rw);
        Translation2d backLeft = new Translation2d(x - rl, y - rw);

        return new Translation2d[] {frontRight, frontLeft, backRight, backLeft};
    }
}
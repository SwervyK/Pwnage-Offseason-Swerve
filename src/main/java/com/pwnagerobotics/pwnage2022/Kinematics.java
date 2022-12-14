package com.pwnagerobotics.pwnage2022;

import org.ejml.simple.SimpleMatrix;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.geometry.Twist2d;

/**
 * Provides forward and inverse kinematics equations for the robot modeling the wheelbase as a differential drive (with
 * a corrective factor to account for skidding).
 */
// INFO means reference
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
    private static final double L = Constants.kDriveLength;
    private static final double W = Constants.kDriveWidth;
    private static final double R = Math.hypot(L, W);

    //**************************************************************************************************************************************//
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
     * @param wheel_speeds
     * @param wheel_azimuths
     * @return Twist2d representing forward, strafe, and angular velocities in real world units
     */
    public static Twist2d forwardKinematics(double[] wheel_speeds, Rotation2d[] wheel_azimuths) {
        double[] vx = new double[4]; // wheel velocities in the x (forward) direction
        double[] vy = new double[4]; // wheel velocities in the y (strafe) direction

        for (int i = 0; i < vx.length; i++) {
            vx[i] = wheel_azimuths[i].cos() * wheel_speeds[i];
            vy[i] = wheel_azimuths[i].sin() * wheel_speeds[i];
        }

        // average possible solutions to minimize error
        double A = (vy[2] + vy[3]) / 2;
        double B = (vy[0] + vy[1]) / 2;
        double C = (vx[0] + vx[3]) / 2;
        double D = (vx[1] + vx[2]) / 2;

        // average possible solutions to minimize error
        double forward = (C + D) / 2;
        double strafe = (A + B) / 2;
        double rotation = (((strafe - A) * R / L) + ((B - strafe) * R / L) + ((forward - C) * R / W)
                + ((D - forward) * R / W)) / 4;

        return new Twist2d(forward, strafe, rotation);
    }

    /**
     * Use Gyro for dtheta
     */
    public static Twist2d forwardKinematics(double[] wheel_speeds, Rotation2d[] wheel_azimuths, Rotation2d prev_heading, Rotation2d current_heading, double dt) {
        Twist2d ret_val = forwardKinematics(wheel_speeds, wheel_azimuths);
        return new Twist2d(ret_val.dx, ret_val.dy, prev_heading.inverse().rotateBy(current_heading).getRadians() / dt);
    }

    //**************************************************************************************************************************************//

    /**
     * For convenience, integrate forward kinematics with a Twist2d and previous rotation.
     */
    public static Pose2d integrateForwardKinematics(Pose2d current_pose, Twist2d forward_kinematics) {
        return current_pose.transformBy(Pose2d.exp(forward_kinematics));
    }

    // /**
    //  * For convenience, integrate forward kinematics with a Twist2d and previous rotation.
    //  */
    // public static Pose2d integrateForwardKinematics(Pose2d current_pose, Twist2d forward_kinematics) {
    //     return current_pose.transformBy(new Pose2d(forward_kinematics.dx, forward_kinematics.dy,
    //             Rotation2d.fromRadians(forward_kinematics.dtheta)));
    // }

    //**************************************************************************************************************************************//

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












    // WORKS //


    public static Object[][] inverseKinematics(double[] controllerInputs, double gyroHeadingRadians, boolean field_relative, double[] robotCenterDisplacement) {
        return inverseKinematics(controllerInputs[0], controllerInputs[1], controllerInputs[2], gyroHeadingRadians, field_relative, true, robotCenterDisplacement);
    }

    public static Object[][] inverseKinematics(double forward, double strafe, double rotation, double gyroHeadingRadians, boolean field_relative, double[] robotCenterDisplacement) {
        return inverseKinematics(forward, strafe, rotation, gyroHeadingRadians, field_relative, true, robotCenterDisplacement);
    }

    public static Object[][] inverseKinematics(double forward, double strafe, double rotation, double gyroHeadingRadians, boolean field_relative,
                                                boolean normalize_outputs, double[] robotCenterDisplacement) {
        if (field_relative) {
            double temp = forward * Math.cos(gyroHeadingRadians) + strafe * Math.sin(gyroHeadingRadians);
            strafe = -forward * Math.sin(gyroHeadingRadians) + strafe * Math.cos(gyroHeadingRadians);
            forward = temp;
        }

        double A = strafe - rotation * L / R + robotCenterDisplacement[0];
        double B = strafe + rotation * L / R + robotCenterDisplacement[0];
        double C = forward - rotation * W / R + robotCenterDisplacement[1];
        double D = forward + rotation * W / R + robotCenterDisplacement[1];

        Double[] wheel_speeds = new Double[4];
        wheel_speeds[0] = Math.hypot(B, C);
        wheel_speeds[1] = Math.hypot(B, D);
        wheel_speeds[3] = Math.hypot(A, D);
        wheel_speeds[2] = Math.hypot(A, C);

        // normalize wheel speeds if above 1
        if (normalize_outputs) {
            double max_speed = 1;
            for (int i = 0; i < wheel_speeds.length; i++) {
                if (Math.abs(wheel_speeds[i]) > max_speed) {
                    max_speed = Math.abs(wheel_speeds[i]);
                }
            }

            for (var i = 0; i < wheel_speeds.length; i++) {
                wheel_speeds[i] /= max_speed;
            }
        }

        Rotation2d[] wheel_azimuths = new Rotation2d[4];
        wheel_azimuths[0] = Rotation2d.fromRadians(Math.atan2(B, C));
        wheel_azimuths[1] = Rotation2d.fromRadians(Math.atan2(B, D));
        wheel_azimuths[3] = Rotation2d.fromRadians(Math.atan2(A, D));
        wheel_azimuths[2] = Rotation2d.fromRadians(Math.atan2(A, C));

        return new Object[][]{wheel_speeds, wheel_azimuths};
    }
}
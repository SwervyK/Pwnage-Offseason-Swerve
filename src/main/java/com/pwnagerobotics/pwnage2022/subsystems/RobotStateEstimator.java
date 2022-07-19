package com.pwnagerobotics.pwnage2022.subsystems;

import com.pwnagerobotics.pwnage2022.Kinematics;
import com.pwnagerobotics.pwnage2022.RobotState;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Twist2d;
import com.team254.lib.subsystems.Subsystem;

public class RobotStateEstimator extends Subsystem {
    static RobotStateEstimator mInstance = new RobotStateEstimator();
    private RobotState mRobotState = RobotState.getInstance();
    private Drive mDrive = Drive.getInstance();
    private double[] mDriveEncoderPrevDistances = new double[4];
    private double prev_timestamp_ = -1.0;
    private Rotation2d prev_heading_ = null;
    
    public static RobotStateEstimator getInstance() {
        if (mInstance == null) {
            mInstance = new RobotStateEstimator();
        }
        return mInstance;
    }
    
    private RobotStateEstimator() {}
    
    @Override
    public synchronized void onEnabledLoopStart(double timestamp) {
        mDriveEncoderPrevDistances = mDrive.getEncoderDistances();
        prev_timestamp_ = timestamp;
    }
    
    @Override
    public synchronized void onEnabledLoop(double timestamp) {
        // if (prev_heading_ == null) {
        //     prev_heading_ = mRobotState.getLatestFieldToVehicle().getValue().getRotation();
        // }
        // final double dt = timestamp - prev_timestamp_;
        // final double[] distances = mDrive.getEncoderDistances();
        // final double[] delta = new double[mDriveEncoderPrevDistances.length];
        // for (int i = 0; i < delta.length; i++) {
        //     delta[i] = distances[i] - mDriveEncoderPrevDistances[i];
        // }
        // final Rotation2d gyro_angle = mDrive.getRobotAngle();
        // Twist2d odometry_twist;
        // synchronized (mRobotState) {
        //     final Pose2d last_measurement = mRobotState.getLatestFieldToVehicle().getValue();
        //     odometry_twist = Kinematics.forwardKinematics(last_measurement.getRotation(), delta_left,
        //     delta_right, gyro_angle);
        // }
        // final Twist2d measured_velocity = Kinematics.forwardKinematics(
        // delta_left, delta_right, prev_heading_.inverse().rotateBy(gyro_angle).getRadians()).scaled(1.0 / dt);
        // final Twist2d predicted_velocity = Kinematics.forwardKinematics(mDrive.getLeftLinearVelocity(),
        // mDrive.getRightLinearVelocity()).scaled(dt);

        // mRobotState.addObservations(timestamp, odometry_twist, measured_velocity,
        // predicted_velocity);
        // mDriveEncoderPrevDistances = distances;
        // prev_heading_ = gyro_angle;
        // prev_timestamp_ = timestamp;
    }
    @Override
    public void onEnabledLoopStop(double timestamp) { stop(); }
    
    @Override
    public void stop() {}
    
    @Override
    public boolean checkSystem() {
        return true;
    }
    
    @Override
    public void outputTelemetry() { }
}

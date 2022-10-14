package com.pwnagerobotics.pwnage2022.subsystems;

import com.pwnagerobotics.pwnage2022.Constants;
import com.pwnagerobotics.pwnage2022.Kinematics;
import com.pwnagerobotics.pwnage2022.RobotState;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Twist2d;
import com.team254.lib.subsystems.Subsystem;

public class RobotStateEstimator extends Subsystem {
    private static RobotStateEstimator mInstance = null;
    private RobotState mRobotState = RobotState.getInstance();
    private Drive mDrive = Drive.getInstance();
    private double prev_timestamp_ = -1.0;
    private Rotation2d prev_heading_ = null;

    public static RobotStateEstimator getInstance() {
        if (mInstance == null) {
            mInstance = new RobotStateEstimator();
        }

        return mInstance;
    }

    @Override
    public synchronized void onEnabledLoopStart(double timestamp) {
        prev_timestamp_ = timestamp;
    }

    @Override
    public synchronized void onEnabledLoop(double timestamp) {
        if (prev_heading_ == null) {
            prev_heading_ = mRobotState.getLatestFieldToVehicle().getValue().getRotation();
        }
        final double dt = timestamp - prev_timestamp_;
        final double[] wheel_speeds = mDrive.getModuleVelocities();
        final Rotation2d[] wheel_azimuths = mDrive.getModuleRotations();
        final Rotation2d gyro_angle = new Rotation2d(mDrive.getGyro(), false);
        Twist2d odometry_twist;
        synchronized (mRobotState) {
            final Pose2d last_measurement = mRobotState.getLatestFieldToVehicle().getValue();

            // this should be used for debugging forward kinematics without gyro (shouldn't be used in actual code)
            // odometry_twist = Kinematics.forwardKinematics(wheel_speeds, wheel_azimuths).scaled(dt);

            // this should be used for more accurate measurements for actual code
            odometry_twist = Kinematics.forwardKinematics(wheel_speeds,
                    wheel_azimuths, last_measurement.getRotation(), gyro_angle, dt).scaled(dt);
        }

        mRobotState.addVehicleToTurretObservation(timestamp,
        new Pose2d(Constants.kVehicleToTurretTranslation, Rotation2d.fromDegrees(Turret.getInstance().getPosition())));

        final Twist2d measured_velocity = Kinematics.forwardKinematics(
                wheel_speeds, wheel_azimuths, prev_heading_, gyro_angle, dt);
        mRobotState.addObservations(timestamp, odometry_twist, measured_velocity);

        prev_heading_ = gyro_angle;
        prev_timestamp_ = timestamp;
    }

    @Override
    public void onEnabledLoopStop(double timestamp) {

    }
    

    @Override
    public void stop() {}

    @Override
    public boolean checkSystem() {
        return true;
    }

    @Override
    public void outputTelemetry() {
        mRobotState.outputToSmartDashboard();
    }
}
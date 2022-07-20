package com.pwnagerobotics.pwnage2022;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.util.InterpolatingDouble;
import com.team254.lib.util.InterpolatingTreeMap;
import edu.wpi.first.wpilibj.Timer;

import java.util.Map;

public class RobotState {
    
    public static RobotState mInstance;
    public synchronized static RobotState getInstance() {
        if (mInstance == null) mInstance = new RobotState();
        return mInstance;
    }

    private static final int kObservationBufferSize = 100;
    
    private InterpolatingTreeMap<InterpolatingDouble, Pose2d> mFieldToVehicle;
    private Translation2d mVehicleVelocityPredicted;
    private Pose2d mPredictedVehiclePosition;
    private Translation2d mVehicleVelocityMeasured;
    private double mDistanceDriven;

    private RobotState() {
        reset(0.0, Pose2d.identity());
    }

    public synchronized void reset(double startTime, Pose2d initialFieldToVehicle) {
        mFieldToVehicle = new InterpolatingTreeMap<>(kObservationBufferSize);
        mFieldToVehicle.put(new InterpolatingDouble(startTime), initialFieldToVehicle);
        mVehicleVelocityPredicted = Translation2d.identity();
        mPredictedVehiclePosition = Pose2d.identity();
        mVehicleVelocityMeasured = Translation2d.identity();
        mDistanceDriven = 0.0;
    }

    public synchronized void reset() {
        reset(Timer.getFPGATimestamp(), Pose2d.identity());
    }

    /**
     * Returns the robot's position on the field at a certain time. Linearly interpolates between stored robot positions
     * to fill in the gaps.
     */
    public synchronized Pose2d getFieldToVehicle(double timestamp) {
        return mFieldToVehicle.getInterpolated(new InterpolatingDouble(timestamp));
    }

    public synchronized Map.Entry<InterpolatingDouble, Pose2d> getLatestFieldToVehicle() {
        return mFieldToVehicle.lastEntry();
    }

    public synchronized Pose2d getPredictedFieldToVehicle(double lookahead_time) {
        return mPredictedVehiclePosition;
    }

    public synchronized void addFieldToVehicleObservation(double timestamp, Pose2d observation) {
        mFieldToVehicle.put(new InterpolatingDouble(timestamp), observation);
    }

    public synchronized void addObservations(double timestamp, Pose2d displacement, Pose2d currentPosition,
                                                Pose2d predictedPosition) {
        mDistanceDriven += Math.sqrt(Math.pow(currentPosition.getTranslation().x(), 2) + Math.pow(currentPosition.getTranslation().y(), 2));
        addFieldToVehicleObservation(timestamp, currentPosition);
        mVehicleVelocityMeasured = new Translation2d(Math.pow(currentPosition.getTranslation().x(), 2), Math.pow(currentPosition.getTranslation().y(), 2));
        mVehicleVelocityPredicted = new Translation2d(Math.pow(predictedPosition.getTranslation().x(), 2), Math.pow(predictedPosition.getTranslation().y(), 2));
        mPredictedVehiclePosition = predictedPosition;
    }

    public synchronized double getDistanceDriven() {
        return mDistanceDriven;
    }

    public synchronized void resetDistanceDriven() {
        mDistanceDriven = 0.0;
    }

    public synchronized Translation2d getPredictedVelocity() {
        return mVehicleVelocityPredicted;
    }

    public synchronized Translation2d getMeasuredVelocity() {
        return mVehicleVelocityMeasured;
    }
}

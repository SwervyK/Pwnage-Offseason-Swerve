package com.pwnagerobotics.pwnage2022.subsystems;

import com.pwnagerobotics.pwnage2022.Kinematics;
import com.pwnagerobotics.pwnage2022.RobotState;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.subsystems.Subsystem;

public class RobotStateEstimator extends Subsystem {
    static RobotStateEstimator mInstance = new RobotStateEstimator();
    private RobotState mRobotState = RobotState.getInstance();
    private Drive mDrive = Drive.getInstance();
    private double mPrevTimestamp = -1.0;
    private Pose2d mPreviousState = Pose2d.identity();
    
    public static RobotStateEstimator getInstance() {
        if (mInstance == null) {
            mInstance = new RobotStateEstimator();
        }
        return mInstance;
    }
    
    private RobotStateEstimator() {}
    
    @Override
    public synchronized void onEnabledLoopStart(double timestamp) {
        mPrevTimestamp = timestamp;
    }
    
    @Override
    public synchronized void onEnabledLoop(double timestamp) {
        // final Translation2d[] moduleState = new Translation2d[] {
        //     mDrive.getModuleState(0),
        //     mDrive.getModuleState(1),
        //     mDrive.getModuleState(2),
        //     mDrive.getModuleState(3)
        // };
        // final double dt = timestamp - mPrevTimestamp;
        // final Pose2d currentState = Kinematics.forwardKinematics(moduleState);
        // final Pose2d displacement = currentState.transformBy(mPreviousState.inverse());
        // final Pose2d predictedState = currentState.interpolate(displacement, dt);

        // mRobotState.addObservations(timestamp, displacement, currentState, predictedState);
        // mPrevTimestamp = timestamp;
        // mPreviousState = currentState;
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

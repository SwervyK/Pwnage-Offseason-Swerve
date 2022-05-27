package com.pwnagerobotics.lib.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.team254.lib.drivers.LazySparkMax;
import com.team254.lib.drivers.SparkMaxFactory;
import com.team254.lib.subsystems.Subsystem;
import com.team254.lib.util.Logger;
import com.team254.lib.util.Util;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SparkRpmSubsystem extends Subsystem {

    // Recommend initializing in a static block!
    public static class SparkRpmSubsystemConstants {
        public String kName = "ERROR_ASSIGN_A_NAME";

        public int kMasterId = 0;
        public boolean kMasterInverted = false;
        public boolean kSensorInverted = false;
        public double kVelocityConversionFactor = 1.0;
        public SparkSubsystemFollowerConstants[] kFollowers = new SparkSubsystemFollowerConstants[0];

        public double kClosedLoopRampRate = 0.0;

        public double kP = 0;
        public double kI = 0;
        public double kD = 0;
        public double kF = 0;
        public double kF_reverse = 0;
        public double kIZone = 0;

        public double kIdleRpm = 0.0;
        public double kClosedLoopAllowableError = 0.0; // pid will try to get within this amount
        public double kAtSpeedAllowableError = 0.0; // 

        public double kVoltageCompensation = 12.0;
        public double kMaxOutput = 0;
        public double kMinOutput = 0;
    }   

    protected final SparkRpmSubsystemConstants mConstants;
    protected final LazySparkMax mMaster;
    protected final LazySparkMax[] mFollowers; 

    protected final SparkMaxPIDController mPidController;
    protected final RelativeEncoder mSensor;

    private boolean TUNING_MODE = false; 

    protected SparkRpmSubsystem(final SparkRpmSubsystemConstants constants) {

        mConstants = constants;
        mMaster = SparkMaxFactory.createDefaultSparkMax(mConstants.kMasterId);
        mMaster.restoreFactoryDefaults();
        mFollowers = new LazySparkMax[mConstants.kFollowers.length];
        for (int i = 0; i < mFollowers.length; ++i) {
                mFollowers[i] = SparkMaxFactory.createPermanentSlaveSparkMax(
                    mConstants.kFollowers[i].id,
                    mMaster,
                    mConstants.kFollowers[i].invert_motor);
                //mFollowers[i].setIdleMode(IdleMode.kCoast);
        }

        mMaster.enableVoltageCompensation(mConstants.kVoltageCompensation);
        mMaster.setClosedLoopRampRate(mConstants.kClosedLoopRampRate);
        mMaster.setInverted(mConstants.kMasterInverted);
        mMaster.setIdleMode(IdleMode.kCoast);

        mSensor = mMaster.getEncoder();
        //mSensor.setInverted(mConstants.kSensorInverted);
        mSensor.setVelocityConversionFactor(mConstants.kVelocityConversionFactor);

        mPidController = mMaster.getPIDController();
        mPidController.setP(mConstants.kP);
        mPidController.setI(mConstants.kI);
        mPidController.setD(mConstants.kD);
        mPidController.setIZone(mConstants.kIZone);
        mPidController.setFF(mConstants.kF);
        mPidController.setOutputRange(mConstants.kMinOutput, mConstants.kMaxOutput);
        mPidController.setFeedbackDevice(mSensor);

        mMaster.burnFlash();

        stop();
    }

    private ControlState mControlState;

    private enum ControlState {
        VELOCITY, OPEN_LOOP
    }
    
    public static class PeriodicIO {
        public double timestamp;
        // INPUTS
        public double observedRpm;
        public double rpmError;
        public double observedOutputPercent;
        public double masterTemp;
        public double masterCurrent;
        
        // OUTPUTS
        public double openLoopDemand;
        public double wantedRpm;
        public boolean isAtSpeed;
    }
    
    protected PeriodicIO mPeriodicIO = new PeriodicIO();
    protected Logger<PeriodicIO> mLogger = null;

    @Override
    public synchronized void readPeriodicInputs() {
        mPeriodicIO.timestamp = Timer.getFPGATimestamp();

        mPeriodicIO.observedRpm = mMaster.getEncoder().getVelocity();
        mPeriodicIO.rpmError = mPeriodicIO.observedRpm - mPeriodicIO.wantedRpm;

        mPeriodicIO.observedOutputPercent = mMaster.getAppliedOutput();

        mPeriodicIO.masterTemp = mMaster.getMotorTemperature();
        mPeriodicIO.masterCurrent = mMaster.getOutputCurrent();

        if (mLogger != null) {
            mLogger.add(mPeriodicIO);
        }
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        

        var forwardFF = mConstants.kF;
        var reverseFF = mConstants.kF_reverse;
        if (TUNING_MODE) {
            mPidController.setP(SmartDashboard.getNumber(mConstants.kName + "/kP", mConstants.kP));
            mPidController.setI(SmartDashboard.getNumber(mConstants.kName + "/kI", mConstants.kI));
            mPidController.setD(SmartDashboard.getNumber(mConstants.kName + "/kD", mConstants.kD));
            forwardFF = SmartDashboard.getNumber(mConstants.kName + "/kF", mConstants.kF);
            reverseFF = SmartDashboard.getNumber(mConstants.kName + "/kF_reverse", mConstants.kF_reverse);
            mPidController.setIZone(SmartDashboard.getNumber(mConstants.kName + "/kIZone", mConstants.kIZone));
            mMaster.setClosedLoopRampRate(SmartDashboard.getNumber(mConstants.kName + "/kClosedLoopRampRate", mConstants.kClosedLoopRampRate));
            // if (mPeriodicIO.openLoopDemand == 0)
            //     setWantedRpm(SmartDashboard.getNumber(mConstants.kName + "/Wanted RPM", 0));
            // else 
            // { 
            //     setOpenLoop(mPeriodicIO.openLoopDemand);
            //     SmartDashboard.putNumber(mConstants.kName + "/Wanted RPM", 0);
            // }
        }
        
        mPeriodicIO.isAtSpeed = mPeriodicIO.wantedRpm > 0 && Util.epsilonEquals(mPeriodicIO.wantedRpm, mPeriodicIO.observedRpm, mConstants.kAtSpeedAllowableError);
        
        var actualff = mPeriodicIO.rpmError > 100 ? reverseFF : forwardFF;
        mPidController.setFF(actualff); // if we're trying to slow down, zero feed foward
        if (mControlState == ControlState.VELOCITY) {
            // SmartDashboard.putNumber(mConstants.kName + "/Actual FF", actualff);
            mMaster.set(ControlType.kVelocity, mPeriodicIO.wantedRpm);
        } else if (mControlState == ControlState.OPEN_LOOP) {
            mMaster.set(ControlType.kDutyCycle, mPeriodicIO.openLoopDemand);
        }
    }

    public synchronized void handleMasterReset(boolean reset) {
    }

    @Override
    public void onEnabledLoopStart(double timestamp){

    }

        
    @Override
    public void onEnabledLoop(double timestamp){
        
    }

        
    @Override
    public void onEnabledLoopStop(double timestamp){
        stop();
    }

    protected void setTuningMode(boolean wantsTuningMode){
        TUNING_MODE = wantsTuningMode;

        SmartDashboard.putNumber(mConstants.kName + "/Wanted RPM", mConstants.kIdleRpm);
        SmartDashboard.putNumber(mConstants.kName + "/kP", mConstants.kP);
        SmartDashboard.putNumber(mConstants.kName + "/kI", mConstants.kI);
        SmartDashboard.putNumber(mConstants.kName + "/kD", mConstants.kD);
        SmartDashboard.putNumber(mConstants.kName + "/kF", mConstants.kF);
        SmartDashboard.putNumber(mConstants.kName + "/kF_reverse", mConstants.kF_reverse);
        SmartDashboard.putNumber(mConstants.kName + "/kIZone", mConstants.kIZone);
        SmartDashboard.putNumber(mConstants.kName + "/kClosedLoopRampRate", mConstants.kClosedLoopRampRate);
    }

    public synchronized double getCurrentRpm() {
        return mPeriodicIO.observedRpm;
    }

    public synchronized double getWantedRpm() {
        return mPeriodicIO.wantedRpm;
    }

    public synchronized boolean isAtSpeed(){
        return mPeriodicIO.isAtSpeed;
    }

    public synchronized double getCurrent() {
        return mPeriodicIO.masterCurrent;
    }

    public synchronized double getMotorTemperature() {
        return mPeriodicIO.masterTemp;
    }

    public synchronized void setWantedRpm(double rpm){
        if (mControlState != ControlState.VELOCITY)
            mControlState = ControlState.VELOCITY;

        // Keep the position setpoint within soft limits
        mPeriodicIO.wantedRpm = rpm;        
    }

    public synchronized void setOpenLoop(double demand) {
        if (mControlState != ControlState.OPEN_LOOP) {
            mControlState = ControlState.OPEN_LOOP;
        }

        mPeriodicIO.openLoopDemand = demand;
    }

    @Override
    public void stop() {
        setWantedRpm(0.0);
    }

    @Override
    public boolean checkSystem() {
        
        return true;
    }

    @Override
    public void outputTelemetry() {
        if (mLogger != null) {
            mLogger.write();
        }

        if (!TUNING_MODE)
            SmartDashboard.putNumber(mConstants.kName + "/Wanted RPM", mPeriodicIO.wantedRpm);
            
        // SmartDashboard.putNumber(mConstants.kName + "/Output Percent", mPeriodicIO.observedOutputPercent);
        // SmartDashboard.putString(mConstants.kName + "/ControlState", mControlState.toString());
        // SmartDashboard.putNumber(mConstants.kName + "/Wanted Demand", mPeriodicIO.openLoopDemand);
        SmartDashboard.putNumber(mConstants.kName + "/RPM Actual", mPeriodicIO.observedRpm);
        SmartDashboard.putNumber(mConstants.kName + "/RPM Error", mPeriodicIO.rpmError);
        // SmartDashboard.putBoolean(mConstants.kName + "/Is At Speed", mPeriodicIO.isAtSpeed);
    }
}
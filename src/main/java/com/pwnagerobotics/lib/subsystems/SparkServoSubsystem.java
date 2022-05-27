package com.pwnagerobotics.lib.subsystems;

import com.revrobotics.SparkMaxAnalogSensor;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.SparkMaxAnalogSensor.Mode;
import com.team254.lib.drivers.LazySparkMax;
import com.team254.lib.drivers.SparkMaxFactory;
import com.team254.lib.subsystems.Subsystem;
import com.team254.lib.util.SynchronousPIDF;
import com.team254.lib.util.Util;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Abstract base class for a subsystem with a single sensored servo-mechanism.
 */
public abstract class SparkServoSubsystem extends Subsystem {

    // Recommend initializing in a static block!
    public static class SparkServoSubsystemConstants {
        public String kName = "ERROR_ASSIGN_A_NAME";

        public int kMasterId = 0;
        public boolean kMasterInverted = false;
        public SparkSubsystemFollowerConstants[] kFollowers = new SparkSubsystemFollowerConstants[0];

        public boolean kSensorInverted = false;
        public double kPositionConversionFactor = 1.0;
        public double kVelocityConversionFactor = 1.0;
        public double kPositionZeroOffset = 0; // position at zero volts

        public double kClosedLoopRampRate = 0.0;
        public double kOpenLoopRampRate = 0.0;

        public double kP = 0;
        public double kI = 0;
        public double kD = 0;
        public double kF = 0;
        public double kIMax = 0;
        public double kPidDeadband = 0;

        public double kHomePosition = 0.0; // Units
        public double kClosedLoopAllowableError = 0.0; // motors will continue until this close +/-
        public double kOnPositionAllowalbeError = 0.0; // subsystem will report on position within this tolerance +/-

        public double kVoltageCompensation = 12.0;
        public double kMinOutput = 0;
        public double kMaxOutput = 0;

        // public double kMaxVelocity = 0; // units
        // public double kMinVelocity = 0; // units / s
        // public double kMaxAcceleration = 0; // units / s / s

        public double kPositionSoftLimitForward = Double.POSITIVE_INFINITY;
        public double kPositionSoftLimitReverse = Double.NEGATIVE_INFINITY;
    }

    protected final SparkServoSubsystemConstants mConstants;
    protected final LazySparkMax mMaster;
    protected final LazySparkMax[] mFollowers; 

    protected final SynchronousPIDF mPidController;
    protected final SparkMaxAnalogSensor mSensor;

    private boolean TUNING_MODE = false; // Change in a specific subsystem to allow debug output and PID tuning

    protected SparkServoSubsystem(final SparkServoSubsystemConstants constants) {
        mConstants = constants;
        mMaster = SparkMaxFactory.createDefaultSparkMax(mConstants.kMasterId);

        mMaster.enableVoltageCompensation(mConstants.kVoltageCompensation);
        mMaster.setClosedLoopRampRate(mConstants.kClosedLoopRampRate);
        mMaster.setOpenLoopRampRate(mConstants.kOpenLoopRampRate);
        mMaster.setInverted(mConstants.kMasterInverted);
        mMaster.setIdleMode(IdleMode.kBrake);

        mFollowers = new LazySparkMax[mConstants.kFollowers.length];
        for (int i = 0; i < mFollowers.length; ++i) {
                mFollowers[i] = SparkMaxFactory.createPermanentSlaveSparkMax(
                    mConstants.kFollowers[i].id,
                    mMaster,
                    mConstants.kFollowers[i].invert_motor);
                //mFollowers[i].setIdleMode(IdleMode.kCoast);
        }

        mSensor = mMaster.getAnalog(Mode.kAbsolute);
        mSensor.setInverted(mConstants.kSensorInverted);
        mSensor.setPositionConversionFactor(mConstants.kPositionConversionFactor);
        mSensor.setVelocityConversionFactor(mConstants.kVelocityConversionFactor);

        mMaster.enableSoftLimit(SoftLimitDirection.kForward, false);
        mMaster.enableSoftLimit(SoftLimitDirection.kReverse, false);
        // mMaster.setSoftLimit(SoftLimitDirection.kForward, (float) (mConstants.kPositionSoftLimitForward + mConstants.kPositionZeroOffset));
        // mMaster.setSoftLimit(SoftLimitDirection.kReverse, (float) (mConstants.kPositionSoftLimitReverse + mConstants.kPositionZeroOffset));

        mPidController = new SynchronousPIDF(
            mConstants.kP, 
            mConstants.kI,
            mConstants.kD,
            mConstants.kF
        );
        mPidController.setIntegratorMax(mConstants.kIMax);
        mPidController.setInputRange(mConstants.kPositionSoftLimitReverse, mConstants.kPositionSoftLimitForward);
        mPidController.setOutputRange(mConstants.kMinOutput, mConstants.kMaxOutput);
        mPidController.setDeadband(mConstants.kPidDeadband);
        
        mMaster.getPIDController().setOutputRange(mConstants.kMinOutput, mConstants.kMaxOutput);
        mMaster.burnFlash();

        // Send a neutral command.
        stop();
    }

    private ControlState mControlState;

    private enum ControlState {
        POSITION, OPEN_LOOP
    }

    public static class PeriodicIO {
        // INPUTS
        public double timestamp;
        public double observed_position;
        // public double observed_velocity;
        public int masterTemp;
        public int masterCurrent;
        public double observed_demand;
        public double positionError;

        // OUTPUTS
        public double open_loop_demand;
        public double wanted_position;
        public boolean isAtPosition;
    }

    protected PeriodicIO mPeriodicIO = new PeriodicIO();
    // protected Logger<PeriodicIO> mCSVWriter = null;
    protected boolean mHasBeenZeroed = false;

    @Override
    public synchronized void readPeriodicInputs() {
        mPeriodicIO.timestamp = Timer.getFPGATimestamp();

        mPeriodicIO.observed_position = mSensor.getPosition() - mConstants.kPositionZeroOffset;
        // mPeriodicIO.observed_velocity = mSensor.getVelocity(); // break these out if we need to use different sensors
        mPeriodicIO.masterTemp = (int) mMaster.getMotorTemperature();
        mPeriodicIO.masterCurrent = (int) mMaster.getOutputCurrent();

        mPeriodicIO.observed_demand = mMaster.getAppliedOutput();
        mPeriodicIO.positionError = mPeriodicIO.observed_position - mPeriodicIO.wanted_position;


        mPeriodicIO.isAtPosition = Util.epsilonEquals(mPeriodicIO.wanted_position, mPeriodicIO.observed_position, mConstants.kOnPositionAllowalbeError);

        // if (mCSVWriter != null) {
        //     mCSVWriter.add(mPeriodicIO);
        // }
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        if (TUNING_MODE) {

            mPidController.setPIDF(
                SmartDashboard.getNumber(mConstants.kName + "/kP", mConstants.kP),
                SmartDashboard.getNumber(mConstants.kName + "/kI", mConstants.kI),
                SmartDashboard.getNumber(mConstants.kName + "/kD", mConstants.kD),
                SmartDashboard.getNumber(mConstants.kName + "/kF", mConstants.kF)
            );

            mPidController.setIntegratorMax(SmartDashboard.getNumber(mConstants.kName + "/kIMax", mConstants.kIMax));
            mPidController.setDeadband(SmartDashboard.getNumber(mConstants.kName + "/kPidDeadband", mConstants.kPidDeadband));
            mPidController.setOutputRange(
                SmartDashboard.getNumber(mConstants.kName + "/kMinOutput", mConstants.kMinOutput), 
                SmartDashboard.getNumber(mConstants.kName + "/kMaxOutput", mConstants.kMaxOutput)
            );
            mMaster.getPIDController().setOutputRange(
                SmartDashboard.getNumber(mConstants.kName + "/kMinOutput", mConstants.kMinOutput), 
                SmartDashboard.getNumber(mConstants.kName + "/kMaxOutput", mConstants.kMaxOutput)
            );

            mMaster.setClosedLoopRampRate(SmartDashboard.getNumber(mConstants.kName + "/Ramp Rate", mConstants.kClosedLoopRampRate));
        }

        var pidIsSatisfied = Util.epsilonEquals(mPeriodicIO.wanted_position, mPeriodicIO.observed_position, mConstants.kClosedLoopAllowableError);

        if (mControlState == ControlState.POSITION && !pidIsSatisfied) {
            mPidController.setSetpoint(mPeriodicIO.wanted_position);
            var output = mPidController.calculate(mPeriodicIO.observed_position);
            if (TUNING_MODE) SmartDashboard.putNumber(mConstants.kName + "/PID output", output);
            mMaster.set(ControlType.kDutyCycle, output);
        } else if (mControlState == ControlState.OPEN_LOOP) {
            if (mPeriodicIO.observed_position >= mConstants.kPositionSoftLimitForward && mPeriodicIO.open_loop_demand > 0) {
                mMaster.set(ControlType.kDutyCycle, 0); 
                // if (mConstants.kName == "Arm")
                    // System.out.println("at forward limit!");
            }
            else if (mPeriodicIO.observed_position <= mConstants.kPositionSoftLimitReverse && mPeriodicIO.open_loop_demand < 0) {
                mMaster.set(ControlType.kDutyCycle, 0);
                // if (mConstants.kName == "Arm")
                    // System.out.println("at reverse limit!");
            }
            else {
                // if (mConstants.kName == "Arm")
                    // System.out.println("setting open loop demand " + mPeriodicIO.open_loop_demand);
                mMaster.set(ControlType.kDutyCycle, mPeriodicIO.open_loop_demand);
            }
        } else { // probably on position
            mMaster.set(ControlType.kDutyCycle, 0);
        }
    }

    public synchronized void handleMasterReset(boolean reset) {
    }

    @Override
    public void onEnabledLoopStart(double timestamp) {
        
    }

    @Override
    public void onEnabledLoop(double timestamp) {
        
    }

    @Override
    public void onEnabledLoopStop(double timestamp) {
        stop();
    }

    @Override
    public void stop(){
        setOpenLoop(0.0);
    }

    // @Override
    // public void registerEnabledLoops(ILooper mEnabledLooper) {
    //     mEnabledLooper.register(new Loop() {
    //         @Override
    //         public void onStart(double timestamp) {
    //             // if (mCSVWriter == null) {
    //             //     mCSVWriter = new Logger<>(
    //             //             mConstants.kName + "/" + mConstants.kName.replaceAll("[^A-Za-z0-9]+", "").toUpperCase()
    //             //                     + "-LOGS_"
    //             //                     + (new Timestamp(System.currentTimeMillis()).toString()).replace(" ", "_"),
    //             //             PeriodicIO.class);
    //             // }
    //         }

    //         @Override
    //         public void onStop(double timestamp) {
    //             // if (mCSVWriter != null) {
    //             //     mCSVWriter.flush();
    //             //     mCSVWriter = null;
    //             // }

    //             stop();
    //         }

    //         @Override
    //         public void onLoop(double timestamp) {
    //         }
    //     });
    // }

    protected void setTuningMode(boolean wantsTuningMode) {
        TUNING_MODE = wantsTuningMode;
        SmartDashboard.putNumber(mConstants.kName + "/Wanted Position", mConstants.kPositionSoftLimitReverse + 1);
        SmartDashboard.putNumber(mConstants.kName + "/kP", mConstants.kP);
        SmartDashboard.putNumber(mConstants.kName + "/kI", mConstants.kI);
        SmartDashboard.putNumber(mConstants.kName + "/kD", mConstants.kD);
        SmartDashboard.putNumber(mConstants.kName + "/kF", mConstants.kF);
        SmartDashboard.putNumber(mConstants.kName + "/kIMax", mConstants.kIMax);
        SmartDashboard.putNumber(mConstants.kName + "/kPidDeadband", mConstants.kPidDeadband);

        SmartDashboard.putNumber(mConstants.kName + "/kMinOutput", mConstants.kMinOutput);
        SmartDashboard.putNumber(mConstants.kName + "/kMaxOutput", mConstants.kMaxOutput);
        SmartDashboard.putNumber(mConstants.kName + "/Ramp Rate", mConstants.kClosedLoopRampRate);

        // SmartDashboard.putNumber(mConstants.kName + "/kMaxVelocity", mConstants.kMaxVelocity);
        // SmartDashboard.putNumber(mConstants.kName + "/kMinVelocity", mConstants.kMinVelocity);
        // SmartDashboard.putNumber(mConstants.kName + "/kMaxAccel", mConstants.kMaxAcceleration);
    }

    // In "Units" due to conversion factor
    public synchronized double getPosition() {
        return mPeriodicIO.observed_position;
    }

    // In "Units per second" due to conversion factor
    // public synchronized double getVelocity() {
    //     return mPeriodicIO.observed_velocity;
    // }

    public synchronized double getWantedPosition() {
        return mPeriodicIO.wanted_position + mConstants.kPositionZeroOffset;
    }

    public synchronized boolean isAtPosition() {
        return mPeriodicIO.isAtPosition;
    }

    public synchronized void setWantedPosition(double units) {
        if (mControlState != ControlState.POSITION)
            mControlState = ControlState.POSITION;

         mPeriodicIO.wanted_position = Util.limit(units, mConstants.kPositionSoftLimitReverse, mConstants.kPositionSoftLimitForward);
         mPeriodicIO.isAtPosition = Util.epsilonEquals(mPeriodicIO.wanted_position, mPeriodicIO.observed_position, mConstants.kOnPositionAllowalbeError);
    }

    public synchronized void setOpenLoop(double percentage) {
        if (mControlState != ControlState.OPEN_LOOP) {
            mControlState = ControlState.OPEN_LOOP;
        }

        // if (mConstants.kName == "Telescope")
        //     System.out.println("setOpenLoop " + percentage);
        mPeriodicIO.open_loop_demand = percentage;
    }

    // Override in implementation
    // public boolean atHomingLocation() {
    //     return false;
    // }
    // public synchronized void resetIfAtLimit() {
    //     if (atHomingLocation()) {
    //         zeroSensors();
    //     }
    // }

    // // Good for internal encoder, override in implementation for other encoder
    // @Override
    // public synchronized void zeroSensors() {
    //     mMaster.getEncoder().setPosition(0);
    //     mHasBeenZeroed = true;
    // }

    // public synchronized boolean hasBeenZeroed() {
    //     return mHasBeenZeroed;
    // }

    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber(mConstants.kName + "/Observed Position", mPeriodicIO.observed_position);
        if (TUNING_MODE) {
            SmartDashboard.putNumber(mConstants.kName + "/Wanted Demand", mPeriodicIO.open_loop_demand);
            SmartDashboard.putNumber(mConstants.kName + "/Observed Demand", mPeriodicIO.observed_demand);
            SmartDashboard.putNumber(mConstants.kName + "/Position Error", mPeriodicIO.positionError);
            SmartDashboard.putBoolean(mConstants.kName + "/At Position", mPeriodicIO.isAtPosition);
        } else {
            SmartDashboard.putNumber(mConstants.kName + "/Wanted Position", mPeriodicIO.wanted_position);
        }
        // synchronized (this) {
        // if (mCSVWriter != null) {
        // mCSVWriter.write();
        // }
        // }
    }
}
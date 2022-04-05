package com.team254.lib.drivers;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxAlternateEncoder.Type;
import com.revrobotics.CANSparkMax;

/**
 * This class is a thin wrapper around the CANTalon that reduces CAN bus / CPU
 * overhead by skipping duplicate set commands.
 */
public class LazySparkMax extends CANSparkMax {
    protected double mLastSet = Double.NaN;
    protected ControlType mLastControlType = null;

    // Set if is a follower
    protected CANSparkMax mLeader = null;

    public LazySparkMax(int deviceNumber) {
        super(deviceNumber, MotorType.kBrushless);
    }

    public CANSparkMax getLeader() {
        return mLeader;
    }

    @Override
    public REVLibError follow(final CANSparkMax leader) {
        mLeader = leader;
        return super.follow(leader);
    }

    /**
     * wrapper method to mimic TalonSRX set method
     */
    public void set(ControlType type, double setpoint) {
        if (setpoint != mLastSet || type != mLastControlType) {
            mLastSet = setpoint;
            mLastControlType = type;
            super.getPIDController().setReference(setpoint, type);
        }
    }

    public void set(ControlType type, double setpoint, int slot) {
        if (setpoint != mLastSet || type != mLastControlType) {
            mLastSet = setpoint;
            mLastControlType = type;
            super.getPIDController().setReference(setpoint, type, slot);
        }
    }

    public void set(ControlType type, double setpoint, int slot, double arbFF) {
        if (setpoint != mLastSet || type != mLastControlType) {
            mLastSet = setpoint;
            mLastControlType = type;
            super.getPIDController().setReference(setpoint, type, slot, arbFF);
        }
    }
}
package com.pwnagerobotics.pwnage2022.lib;

import com.pwnagerobotics.pwnage2022.Constants;
import com.team254.lib.drivers.LazySparkMax;
import com.team254.lib.drivers.SparkMaxFactory;
import com.team254.lib.util.SynchronousPIDF;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule {

  private final boolean DEBUG_MODE = true;
  private final boolean TUNING = false;
  
  public static class SwerveModuleConstants {
    public String kName = "Name";
    public int kDriveId = 0;
    public int kRotationId = 0;
    public int kRotationEncoderId = 0;
    
    public double kp = 0.01;
    public double ki = 0.0;
    public double kd = 0.0;
    
    public double kRotationOffset = 0.0;
    public double kRotationError = 0.5; // *Degrees
    public double kWheelDiameter = 0.0;
  }
  
  private SwerveModuleConstants mConstants;
  private LazySparkMax mDriveController;
  private LazySparkMax mRotationController;
  private RelativeEncoder mDriveEncoder;
  private DutyCycleEncoder mRotationEncoder;
  private SynchronousPIDF mPID;

  private double mLastMagnitude = 0;
  private double mLastDrive = 0;
  private double mLastRotation = 0;
  private boolean mFlipped = false;
  
  public SwerveModule(SwerveModuleConstants constants) {
    mConstants = constants;
    mDriveController = SparkMaxFactory.createDefaultSparkMax(mConstants.kDriveId);
    mDriveController.enableVoltageCompensation(Constants.kDriveVoltageOpenLoopCompSaturation);
    mDriveController.setSmartCurrentLimit(Constants.kDriveCurrentLimit);
    mDriveController.burnFlash();

    mDriveEncoder = mDriveController.getEncoder();

    mRotationController = SparkMaxFactory.createDefaultSparkMax(mConstants.kRotationId);
    mRotationController.enableVoltageCompensation(Constants.kDriveVoltageOpenLoopCompSaturation);
    mRotationController.setSmartCurrentLimit(Constants.kDriveCurrentLimit);
    mRotationController.burnFlash();

    mRotationEncoder = new DutyCycleEncoder(mConstants.kRotationEncoderId);
    mRotationEncoder.setPositionOffset(mConstants.kRotationOffset);
    mRotationEncoder.setDutyCycleRange(0, Constants.kRotationEncoderCPR);

    mPID = new SynchronousPIDF(mConstants.kp, mConstants.ki, mConstants.kd);
    mPID.setInputRange(-180, 180);
    mPID.setOutputRange(-1, 1);
  }
  
  /**
  * Set module speed and rotation
  * @param wantedAngle Module angle (0 is forward)
  * @param magnitude Speed (-1 to 1)
  */
  public void setModuleDegrees(double wantedAngle, double magnitude) { // TODO wantedAngle from -180 to 180
    if (TUNING) tuneRobotRotationPID();
    double currentAngle = SwerveDriveHelper.clamp(getRotationDegrees(), 360, 0, true); // Gets current angle from rotation encoder
    double distance = SwerveDriveHelper.getAngularDistance(currentAngle, wantedAngle, 360);

    if (mConstants.kName.equals("Front Right") && DEBUG_MODE) {
      SmartDashboard.putNumber("Magnitude Initial", magnitude);
      SmartDashboard.putNumber("Controller Initial", wantedAngle);
    }
    // At higher speeds you need larger angles to flip because of the time is takes to reverse the drive direction
    if (Math.abs(getDriveVelocity()) >= Constants.kDrive180Speed) {
      if (mLastMagnitude < 0) magnitude *= -1; // Without this wheels will only move forward regardless of their last direction
      if (mFlipped) {
        wantedAngle = SwerveDriveHelper.clamp(wantedAngle-180, 360, 0, true);
        distance = SwerveDriveHelper.getAngularDistance(currentAngle, wantedAngle, 360);
      }
    }
    else if (Math.abs(distance) > 90) { // Makes sure the robot is taking the most optimal path when rotating modules
      wantedAngle = SwerveDriveHelper.clamp(wantedAngle-180, 360, 0, true);
      distance = SwerveDriveHelper.getAngularDistance(currentAngle, wantedAngle, 360);
      magnitude *= -1;
      mFlipped = true;
    }
    else {
      mFlipped = false;
    }

    if (mConstants.kName.equals("Front Right") && DEBUG_MODE) {
      SmartDashboard.putNumber("Magnitude Final", magnitude);
      SmartDashboard.putNumber("Controller Final", wantedAngle);
    }

    if (magnitude == 0) mDriveController.stopMotor();
    else mDriveController.set(ControlType.kDutyCycle, magnitude * Constants.kDriveSlowDown);
    
    mPID.setSetpoint(distance);
    double rotationSpeed = mPID.calculate(0);
    if (mPID.onTarget(mConstants.kRotationError) || magnitude == 0) mRotationController.set(0);
    else mRotationController.set(ControlType.kDutyCycle, rotationSpeed * Constants.kRotationSlowDown);

    mLastMagnitude = magnitude;
    mLastDrive = getDrive();
    mLastRotation = getRotationDegrees();
  }
  
  public void outputTelemetry() {
    SmartDashboard.putNumber("Current Speed: " + mConstants.kName, mDriveController.get());
    SmartDashboard.putNumber("Current Angle: " + mConstants.kName, getRotationDegrees());
  }
  
  private void tuneRobotRotationPID() {
    if (-1 == SmartDashboard.getNumber("kP", -1)) SmartDashboard.putNumber("kP", 0);
    if (-1 == SmartDashboard.getNumber("kI", -1)) SmartDashboard.putNumber("kI", 0);
    if (-1 == SmartDashboard.getNumber("kD", -1)) SmartDashboard.putNumber("kD", 0);
    mPID.setPID(SmartDashboard.getNumber("kP", 0), SmartDashboard.getNumber("kI", 0), SmartDashboard.getNumber("kD", 0));
  }

  public void setBrake(boolean brake) {
    mDriveController.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
    mRotationController.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
  }
  
  public void zeroEncoders() {
    mDriveEncoder.setPosition(0);
  }

  public void stop() {
    zeroEncoders();
    setBrake(true);
    mPID.reset();
  }

  public double getDrive() {
    return mDriveEncoder.getPosition();
  }

  public double getDriveDelta() {
    return mLastDrive - getDrive();
  }

  public double getDriveVelocity() {
    return mDriveEncoder.getVelocity();
  }

  public double getRotationDegrees() {
    return SwerveDriveHelper.clamp(mRotationEncoder.getAbsolutePosition() - mRotationEncoder.getPositionOffset(), 1, 0, true) * 360;
  }

  public double getRotationDelta() {
    return mLastRotation - getRotationDegrees();
  }
}

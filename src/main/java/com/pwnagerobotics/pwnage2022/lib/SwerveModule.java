package com.pwnagerobotics.pwnage2022.lib;

import com.pwnagerobotics.pwnage2022.Constants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule {
  
  private SwerveModuleConstants mConstants;
  private MotorController mDriveController;
  private MotorController mRotationController;
  // private AnalogEncoder mDrivEncoder;
  private AnalogEncoder mRotationEncoder;
  private PIDController mPID;
  private double mRotationOffset;
  
  private SlewRateLimiter mDriveRateLimiter = new SlewRateLimiter(Constants.kDriveRateLimit);
  private double mLastThrottle = 0;
  private static final double kPIDInputRange = 90;
  
  public SwerveModule(SwerveModuleConstants constants) {
    mConstants = constants;
    mDriveController = new PWMMotorController(mConstants.kName + " Drive",  mConstants.kDriveId) { };
    mRotationController = new PWMMotorController(mConstants.kName + " Rotation", mConstants.kRotationId) { };
    // mDrivEncoder = new AnalogEncoder(mConstants.kDriveEncoderId);
    mRotationEncoder = new AnalogEncoder(mConstants.kRotationEncoderId);
    mRotationOffset = mConstants.kRotationOffset;
    mPID = new PIDController(mConstants.kp, mConstants.ki, mConstants.kd);
    mPID.enableContinuousInput(-kPIDInputRange, kPIDInputRange);
    mPID.setTolerance(mConstants.kRotationError);
  }
  
  public void setModule(double wantedPosition, double throttle)
  {
    // Postion
    double currentPosition = (mRotationEncoder.getAbsolutePosition() - mRotationOffset) * 360;
    currentPosition = clamp(currentPosition, 360, 0, true);
    
    // Distance
    double distance = getDistance(currentPosition, wantedPosition);
    
    // 90 flip
    if (Math.abs(distance) > 90) {
      wantedPosition -= 180;
      wantedPosition = clamp(wantedPosition, 360, 0, true);
      distance = getDistance(currentPosition, wantedPosition);
      throttle *= -1;
    }
    
    // Drive
    //if (Drive.getInstance().getCurrent(mConstants.kPDPId) > Constants.kDriveCurrentLimit) throttle = 0; // Current Limit
    throttle = getAdjustedThrottle(mLastThrottle, throttle); // Ramp rate
    if (throttle == 0) mDriveController.stopMotor();
    else mDriveController.set(throttle * Constants.kDriveSlowDown);
    
    // Rotation
    double rotationSpeed = clamp(mPID.calculate(0, distance), 1, -1, false);
    if (mPID.atSetpoint())
    mRotationController.set(0);
    else
    mRotationController.set(rotationSpeed * Constants.kRotationSlowDown);
    mLastThrottle = throttle;
    
    // Logging
    mDeltaRotationSpeed = mOldRotationSpeed - rotationSpeed;
    if (mDeltaRotationSpeed > mMaxDeltaRotationSpeed) mMaxDeltaRotationSpeed = mDeltaRotationSpeed;
    mOldRotationSpeed = rotationSpeed;
    mDeltaDriveSpeed = mOldDriveSpeed - throttle;
    if (mDeltaDriveSpeed > mMaxDeltaDriveSpeed) mMaxDeltaDriveSpeed = mDeltaDriveSpeed;
    mOldDriveSpeed = throttle;
    mCurrentSpeed = throttle;
    mCurrentAngle = wantedPosition;
  }
  
  private double getAdjustedThrottle(double lastThrottle, double throttle) {
    if (throttle == 0) {
      mDriveRateLimiter.reset(0);
      return 0;
    }
    if (Math.abs(throttle) < Math.abs(lastThrottle) || Math.abs(throttle) < 0.2) {
      mDriveRateLimiter.reset(throttle);
      return throttle;
    }
    return mDriveRateLimiter.calculate(throttle);
  }
  
  private double mOldRotationSpeed = 0;
  private double mDeltaRotationSpeed = 0;
  private double mOldDriveSpeed = 0;
  private double mDeltaDriveSpeed = 0;
  private double mMaxDeltaDriveSpeed = 0;
  private double mMaxDeltaRotationSpeed = 0;
  private double mCurrentSpeed = 0;
  private double mCurrentAngle = 0;
  
  public void outputTelemetry() {
    SmartDashboard.putNumber("Delta Rotation Speed: " + mConstants.kName, mDeltaRotationSpeed);
    SmartDashboard.putNumber("Delta Drive Speed: " + mConstants.kName, mDeltaDriveSpeed);
    SmartDashboard.putNumber("Max Delta Drive Speed: " + mConstants.kName, mMaxDeltaDriveSpeed);
    SmartDashboard.putNumber("Max Delta Rotation Speed: " + mConstants.kName, mMaxDeltaRotationSpeed);
    SmartDashboard.putNumber("Current Speed: " + mConstants.kName, mCurrentSpeed);
    SmartDashboard.putNumber("Current Angle: " + mConstants.kName, mCurrentAngle);
    // mLastDriveValue = mDrivEncoder.getDistance();
  }
  
  public static double clamp(double value, double max, double min, boolean wrapAround) {
    if (wrapAround) {
      if (value > max)
      return (value - (max-min) * ((int)((value-max-1)/(max - min)))) - max + min;
      else if (value < min)
      return value + (max-min) * -((((int)((value-max)/(max - min))))+1) - min + max;
      return value;
    }
    else 
    return (value>=max)?max:(value<=min)?min:value;
  }
  
  // Built in 180 flip
  public static double getDistance(double current, double wanted) {
    double result = current - wanted;
    if (Math.abs(result) > 180) {
      result += 360 * -Math.signum(result);
    }
    return result;
  }
  
  public void setPID(double kp, double ki, double kd) {
    mPID.setPID(kp, ki, kd);
  }
  
  public double getRotation() {
    return mRotationEncoder.getAbsolutePosition() * 360;
  }
  
  public void zeroEncoders() {
    mRotationEncoder.reset();
  }
  
  private void tuneRobotRotationPID() {
    if (-1 == SmartDashboard.getNumber("kP", -1)) SmartDashboard.putNumber("kP", 0);
    if (-1 == SmartDashboard.getNumber("kI", -1)) SmartDashboard.putNumber("kI", 0);
    if (-1 == SmartDashboard.getNumber("kD", -1)) SmartDashboard.putNumber("kD", 0);
    mPID.setPID(SmartDashboard.getNumber("kP", 0), SmartDashboard.getNumber("kI", 0), SmartDashboard.getNumber("kD", 0));
  }
  
  // private double mLastDriveValue;
  // public double getDeltaDrive() {
    //   return mLastDriveValue - mDrivEncoder.getDistance();
    // }
  }
  
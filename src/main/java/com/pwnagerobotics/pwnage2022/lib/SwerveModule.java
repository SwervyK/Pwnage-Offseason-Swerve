package com.pwnagerobotics.pwnage2022.lib;

import com.pwnagerobotics.pwnage2022.Constants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;

public class SwerveModule {
  
  private SwerveModuleConstants mConstants;
  private MotorController mDriveController;
  private MotorController mRotationController;
  private AnalogEncoder mRotationEncoder;
  private PIDController mPID;
  private double mRotationOffset;
  
  private static final double kPIDInputRange = 90;
  
  public SwerveModule(SwerveModuleConstants constants) {
    mConstants = constants;
    mDriveController = new PWMMotorController(mConstants.kName + " Drive",  mConstants.kDriveId) { };
    mRotationController = new PWMMotorController(mConstants.kName + " Rotation", mConstants.kRotationId) { };
    mRotationEncoder = new AnalogEncoder(mConstants.kRotationEncoderId);
    mRotationOffset = mConstants.kRotationOffset;
    // driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter); TODO
    // driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
    // turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
    // turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);
    mPID = new PIDController(mConstants.kp, mConstants.ki, mConstants.kd);
    mPID.enableContinuousInput(-kPIDInputRange, kPIDInputRange);
    mPID.setTolerance(mConstants.kRotationError);
  }

  public void setModule(double rotation, double throttle)
  {
    // Postion
    double currentPosition = (mRotationEncoder.getAbsolutePosition() - mRotationOffset) * 360;
    if (currentPosition < 0) currentPosition += 360;
    double wantedPosition = rotation;
    
    // Distance
    double distance = getDistance(currentPosition, wantedPosition);
    
    // 90 flip
    if (Math.abs(distance) > 90 && Math.abs(distance) < 270) { // Maybe make a smaller range
      wantedPosition -= 180;
      if (wantedPosition < 0) wantedPosition += 360;
      distance = getDistance(currentPosition, wantedPosition);
      throttle *= -1;
    }
    
    // Drive
    mDriveController.set(throttle * Constants.kDriveSlowDown);
    
    // Rotation
    double rotationSpeed = mPID.calculate(0, distance);
    if (rotationSpeed > 1) rotationSpeed = 1;
    if (rotationSpeed < -1) rotationSpeed = -1;
    if (mPID.atSetpoint()) {
      mRotationController.set(0);
    }
    else {
      mRotationController.set(rotationSpeed * Constants.kRotationSlowDown);
    }
  }
  
  public static double getDistance(double encoder, double controller) {
    double result = encoder - controller;
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
}
  
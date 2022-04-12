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
  
  public SwerveModule(SwerveModuleConstants constants) {
    mConstants = constants;
    mDriveController = new PWMMotorController(mConstants.kName + " Drive",  mConstants.kDriveId) { };
    mRotationController = new PWMMotorController(mConstants.kName + " Rotation", mConstants.kRotationId) { };
    mRotationEncoder = new AnalogEncoder(mConstants.kRotationEncoderId);
    mRotationOffset = mConstants.kRotationOffset;
    // mRotationEncoder.setPositionOffset(mRotationOffset);
    mPID = new PIDController(mConstants.kp, mConstants.ki, mConstants.kd);
    mPID.enableContinuousInput(-90, 90);
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
    double rotationSpeed = mPID.calculate(0, -distance)/Constants.kMaxPIDValue * Constants.kTurnSlowDown;
    if (mPID.atSetpoint()) {
      mRotationController.set(0);
    }
    else {
      mRotationController.set(rotationSpeed);
    }
    
    // if (rotationOffset == 0.590) {
      //   SmartDashboard.putNumber("Full Speed", rotationSpeed * kMaxPIDValue);
      //   SmartDashboard.putNumber("Motor Speed", rotationSpeed);
      //   SmartDashboard.putNumber("Wanted Pos", wantedPosition);
      //   SmartDashboard.putNumber("Distance", distance);
      //   SmartDashboard.putBoolean("Is at Pos", pidController.atSetpoint());
      //   SmartDashboard.putNumber("Current Pos", currentPosition);
      //   SmartDashboard.putNumber("Encoder Pos", rotationEncoder.getAbsolutePosition());
      // }
    }
    
    private double getDistance(double encoder, double controller) {
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
  
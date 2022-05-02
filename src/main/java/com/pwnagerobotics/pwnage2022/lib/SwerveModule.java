package com.pwnagerobotics.pwnage2022.lib;

import com.pwnagerobotics.pwnage2022.Constants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;

public class SwerveModule {
  
  private SwerveModuleConstants mConstants;
  private MotorController mDriveController;
  private MotorController mRotationController;
  private AnalogEncoder mRotationEncoder;
  private PIDController mPID;
  private double mRotationOffset;
  private PowerDistribution PDP = new PowerDistribution();
  private SlewRateLimiter mDriveRateLimiter = new SlewRateLimiter(0.5);
  
  private static final double kPIDInputRange = 90;
  
  public SwerveModule(SwerveModuleConstants constants) {
    mConstants = constants;
    mDriveController = new PWMMotorController(mConstants.kName + " Drive",  mConstants.kDriveId) { };
    mRotationController = new PWMMotorController(mConstants.kName + " Rotation", mConstants.kRotationId) { };
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
    currentPosition = clamp(currentPosition, 360, 0, true); //if (currentPosition < 0) currentPosition += 360;
    
    // Distance
    double distance = getDistance(currentPosition, wantedPosition);
    
    // 90 flip
    if (Math.abs(distance) > 90 && Math.abs(distance) < 270) { // Maybe make a smaller range
      wantedPosition -= 180;
      wantedPosition = clamp(wantedPosition, 360, 0, true); //if (wantedPosition < 0) wantedPosition += 360;
      distance = getDistance(currentPosition, wantedPosition);
      throttle *= -1;
    }
    
    // Drive
    //if (PDP.getCurrent(mConstants.kPDPId) > Constants.kDriveCurrentLimit) throttle = 0; //TODO Current Limit
    //throttle = mDriveRateLimiter.calculate(throttle); //TODO Rate Limit
    mDriveController.set(throttle * Constants.kDriveSlowDown);
    
    // Rotation
    double rotationSpeed = clamp(mPID.calculate(0, distance), 1, -1, false);
    // if (rotationSpeed > 1) rotationSpeed = 1;
    // if (rotationSpeed < -1) rotationSpeed = -1;
    if (mPID.atSetpoint()) {
      mRotationController.set(0);
    }
    else {
      mRotationController.set(rotationSpeed * Constants.kRotationSlowDown);
    }
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

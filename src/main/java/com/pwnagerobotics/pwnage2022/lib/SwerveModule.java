package com.pwnagerobotics.pwnage2022.lib;

import com.pwnagerobotics.pwnage2022.Constants;
import com.pwnagerobotics.pwnage2022.Kinematics;
import com.pwnagerobotics.pwnage2022.RobotState;
import com.pwnagerobotics.pwnage2022.subsystems.Drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule {

  public static class SwerveModuleConstants {
    public String kName = "Name";
    public int kDriveId = 0;
    public int kRotationId = 0;
    public int[] kDriveEncoderId = {0, 0};
    public int kRotationEncoderId = 0;
    public int kPDPId = 0;
    
    public double kp = 0.01;
    public double ki = 0.001;
    public double kd = 0.0;
    
    public double kRotationOffset = 0.0;
    public double kRotationError = 3; // Degrees
    public double kWheelDiameter = 0.0;
  }

  private SwerveModuleConstants mConstants;
  private MotorController mDriveController;
  private MotorController mRotationController;
  // private Encoder mDrivEncoder;
  private AnalogEncoder mRotationEncoder;
  private PIDController mPID;
  private double mRotationOffset;
  private RobotState mRobotState = RobotState.getInstance();
  private SlewRateLimiter mDriveRateLimiter = new SlewRateLimiter(Constants.kDriveRateLimit);
  private double mLastThrottle = 0;
  private static final double kPIDInputRange = 180;
  
  public SwerveModule(SwerveModuleConstants constants) {
    mConstants = constants;
    mDriveController = new PWMMotorController(mConstants.kName + " Drive",  mConstants.kDriveId) { };
    mRotationController = new PWMMotorController(mConstants.kName + " Rotation", mConstants.kRotationId) { };
    // mDrivEncoder = new Encoder (mConstants.kDriveEncoderId[0], mConstants.kDriveEncoderId[1]);
    mRotationEncoder = new AnalogEncoder(mConstants.kRotationEncoderId);
    mRotationOffset = mConstants.kRotationOffset;
    mPID = new PIDController(mConstants.kp, mConstants.ki, mConstants.kd);
    mPID.enableContinuousInput(-kPIDInputRange, kPIDInputRange);
    mPID.setTolerance(mConstants.kRotationError);
  }
  
  public void setModule(double wantedPosition, double throttle)
  {
    // Postion
    double currentPosition = (mRotationEncoder.getAbsolutePosition() - mRotationOffset) * 360; // * 360 is to convert from the 0 to 1 of the encoder to 0 to 360
    currentPosition = clamp(currentPosition, 360, 0, true);
    
    // Distance
    double distance = getDistance(currentPosition, wantedPosition);
    
    // 90 flip
    // At higher speeds maybe need larger angles to flip because of the time is takes to reverse the drive direction
    // TODO make it scale based on speed?
    if (!(throttle >= 0.75)) {
      throttle = mLastThrottle; // Without this wheels will only move fowared regardless of their last direction
    }
    else if (Math.abs(distance) > 90 /*|| mRobotState.getMeasuredVelocity().norm() < 5*/) { // Makes sure the robot is takig the most optimal path when rotating modues
      wantedPosition -= 180;
      wantedPosition = clamp(wantedPosition, 360, 0, true);
      distance = getDistance(currentPosition, wantedPosition);
      throttle *= -1;
    }
    
    // Drive
    //if (Drive.getInstance().getCurrent(mConstants.kPDPId) > Constants.kDriveCurrentLimit) throttle = 0; // Current Limit
    // throttle = getAdjustedThrottle(mLastThrottle, throttle); // Ramp rate
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
    mCurrentSpeed = throttle;
    mCurrentAngle = wantedPosition;
  }

  private double mCurrentSpeed = 0;
  private double mCurrentAngle = 0;
  
  public void outputTelemetry() {
    SmartDashboard.putNumber("Current Speed: " + mConstants.kName, mCurrentSpeed);
    SmartDashboard.putNumber("Current Angle: " + mConstants.kName, mCurrentAngle);
    // mLastDriveValue = mDrivEncoder.getDistance();
  }

  // Wraps around the value proportionally between min and max
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
  
  // Ramp rate
  // Slowing down is not limited
  // Dont limit +-0.2 so if you go from 1 to -1 you can get to -0.2 unlimited so make slowing down faster
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
     public double getDriveEncoderDistance() {
       return 0; //mDrivEncoder.getDistance();
     }

     public double getDriveVelocity() {
      return mDriveController.get();
     }

     public double getModuleAngle() {
      return mCurrentAngle;
     }
}
  
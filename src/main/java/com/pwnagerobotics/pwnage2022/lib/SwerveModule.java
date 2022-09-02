package com.pwnagerobotics.pwnage2022.lib;

import com.pwnagerobotics.pwnage2022.Constants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule {

  private final boolean DEBUG_MODE = false;
  
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
    public double kRotationError = 3; // Degrees // TODO tune
    public double kWheelDiameter = 0.0;
  }
  
  private SwerveModuleConstants mConstants;
  private MotorController mDriveController;
  private MotorController mRotationController;
  // private Encoder mDriveEncoder;
  private AnalogEncoder mRotationEncoder;
  private PIDController mPID;
  private SlewRateLimiter mDriveRateLimiter = new SlewRateLimiter(Constants.kDriveRateLimit);
  private double mLastMagnitude = 0;
  // private double mLastDrive = 0;
  private double mLastRotation = 0;
  private boolean mFlipped = false;
  
  public SwerveModule(SwerveModuleConstants constants) {
    mConstants = constants;
    mDriveController = new PWMMotorController(mConstants.kName + " Drive",  mConstants.kDriveId) { };
    mRotationController = new PWMMotorController(mConstants.kName + " Rotation", mConstants.kRotationId) { };
    // mDriveEncoder = new Encoder (mConstants.kDriveEncoderId[0], mConstants.kDriveEncoderId[1]);
    mRotationEncoder = new AnalogEncoder(mConstants.kRotationEncoderId);
    mPID = new PIDController(mConstants.kp, mConstants.ki, mConstants.kd);
    mPID.setTolerance(mConstants.kRotationError);
  }
  
  /**
  * Set module speed and rotation
  * @param wantedAngle Module angle (0 is forward)
  * @param magnitude Speed (-1 to 1)
  */
  public void setModule(double wantedAngle, double magnitude) {
    if (DEBUG_MODE) tuneRobotRotationPID();
    double currentAngle = (mRotationEncoder.getAbsolutePosition() - mConstants.kRotationOffset) * 360; // * 360 is to convert from the 0 to 1 of the encoder to 0 to 360
    currentAngle = Util.clamp(currentAngle, 360, 0, true);
    double distance = Util.getDistance(currentAngle, wantedAngle);
    if (mConstants.kName.equals("Front Right")) {
      SmartDashboard.putNumber("Magnitude Initial", magnitude);
      SmartDashboard.putNumber("Controller Initial", wantedAngle);
    }
    // 90 flip
    // At higher speeds you need larger angles to flip because of the time is takes to reverse the drive direction
    // TODO make it scale based on speed not controller
    // Module Speed or Robot Speed?    
    if (magnitude >= 0.75) {
      if (mLastMagnitude < 0) magnitude *= -1; // Without this wheels will only move forward regardless of their last direction
      if (mFlipped) {
        wantedAngle = Util.clamp(wantedAngle-180, 360, 0, true);
        distance = Util.getDistance(currentAngle, wantedAngle);
      }
    }
    else if (Math.abs(distance) > 90) { // Makes sure the robot is taking the most optimal path when rotating modules
      wantedAngle = Util.clamp(wantedAngle-180, 360, 0, true);
      distance = Util.getDistance(currentAngle, wantedAngle);
      magnitude *= -1;
      mFlipped = true;
    }
    else {
      mFlipped = false;
    }

    // if (Drive.getInstance().getCurrent(mConstants.kPDPId) > Constants.kDriveCurrentLimit) throttle = 0; // Current Limit
    // throttle = getAdjustedThrottle(mLastThrottle, throttle); // Ramp rate
    if (mConstants.kName.equals("Front Right")) {
      SmartDashboard.putNumber("Magnitude Final", magnitude);
      SmartDashboard.putNumber("Controller Final", wantedAngle);
    }
    if (magnitude == 0) mDriveController.stopMotor();
    else mDriveController.set(magnitude * Constants.kDriveSlowDown);
    
    double rotationSpeed = Util.clamp(mPID.calculate(0, distance), 1, -1, false);
    if (mPID.atSetpoint()) mRotationController.set(0);
    else mRotationController.set(rotationSpeed * Constants.kRotationSlowDown);

    mLastMagnitude = magnitude;
    // mLastDrive = getDrive();
    mLastRotation = getRotation();
  }
  
  public void outputTelemetry() {
    SmartDashboard.putNumber("Current Speed: " + mConstants.kName, mDriveController.get());
    SmartDashboard.putNumber("Current Angle: " + mConstants.kName, (mRotationEncoder.getAbsolutePosition() - mConstants.kRotationOffset) * 360);
  }

  // Ramp rate
  // Slowing down is not limited
  // from -0.2 to 0.2 is not limited
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
  
  public void zeroEncoders() {
    mRotationEncoder.reset();
    // mDriveEncoder.reset();
  }

  public double getDrive() {
    return 0; // mDriveEncoder.getDistance();
  }

  public double getDriveDelta() {
    return 0; // mLastDrive - getDrive();
  }

  public double getDriveVelocity() {
    return mDriveController.get();
  }

  public double getRotation() {
    return mRotationEncoder.getAbsolutePosition() * 360;
  }

  public double getRotationDelta() {
    return mLastRotation - getRotation();
  }
  
  public double getRotationVelocity() {
    return mRotationController.get();
  }
  
}

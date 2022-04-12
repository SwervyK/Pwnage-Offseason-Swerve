package com.pwnagerobotics.pwnage2022.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pwnagerobotics.pwnage2022.Constants;
import com.pwnagerobotics.pwnage2022.lib.SwerveModule;
import com.pwnagerobotics.pwnage2022.subsystems.Drive;

import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.subsystems.Subsystem;

public class Drive extends Subsystem {
  // Singleton Drive
  public static Drive mInstance;
  public synchronized static Drive getInstance() {
    if (mInstance == null) mInstance = new Drive();
    return mInstance;
  }
  
  public enum DriveMode {
    ROBOT,
    FEILD
  }
  private DriveMode mCurrentDriveMode = DriveMode.ROBOT;
  public enum RotationMode {
    ROBOT,
    FEILD
  }
  private RotationMode mCurrentRotationMode = RotationMode.ROBOT;
  private static final double kControllerDeadband = 0.05;
  private static SwerveModule[] mModules;
  // private static AHRS mNavX = new AHRS();
  
  
  public Drive() {
    mModules[0] = new SwerveModule(Constants.kFrontRightModuleConstants);
    mModules[1] = new SwerveModule(Constants.kFrontLefttModuleConstants);
    mModules[2] = new SwerveModule(Constants.kBackRightModuleConstants);
    mModules[3] = new SwerveModule(Constants.kBackLefttModuleConstants);
    // Rotation2d mGyroOffset = Rotation2d.identity().rotateBy(Rotation2d.fromDegrees(mNavX.getYaw()).inverse());
    // mNavX.setAngleAdjustment(mGyroOffset.getDegrees());
    // mNavX.setAngleAdjustment(-Constants.kGyroOffset);
    SmartDashboard.putNumber("kp", 0.7);
    SmartDashboard.putNumber("ki", 0.015);
    SmartDashboard.putNumber("kd", 0);
  }
  
  public void updatePID() {
    double kp = SmartDashboard.getNumber("kp", 0);
    double ki = SmartDashboard.getNumber("ki", 0);
    double kd = SmartDashboard.getNumber("kd", 0);
    mModules[0].setPID(kp, ki, kd);
    mModules[1].setPID(kp, ki, kd);
    mModules[2].setPID(kp, ki, kd);
    mModules[3].setPID(kp, ki, kd);
  }
  
  public synchronized void setSwerveDrive(double throttle, double strafe, double rotation) {
    double angle = Math.toDegrees(Math.atan2(strafe, throttle));
    angle = (angle >= 0) ? angle : angle + 360;
    double speed = Math.sqrt(Math.pow(Math.abs(strafe), 2) + Math.pow(Math.abs(throttle), 2));
    speed = (speed >= 1) ? 1 : speed;
    SmartDashboard.putNumber("Controller angle", angle);
    if (Math.abs(speed) < kControllerDeadband) {
      speed = 0;
      angle = 0;
    }

    if (mCurrentDriveMode == DriveMode.FEILD) {
      rotation -= getGyroAngle();
      if (rotation < 0) rotation += 360;
    }

    updatePID();
    mModules[0].setModule(angle, speed);
    mModules[1].setModule(angle, speed);
    mModules[2].setModule(angle, speed);
    mModules[3].setModule(angle, speed);
  }
  
  
  public void setVectorSwerveDrive(double forwardSpeed, double rotationSpeed, double robotAngle){
    Vector2d FRVector, FLVector, BRVector, BLVector;
    FRVector = addMovementComponents(forwardSpeed, robotAngle, rotationSpeed, 135);
    FLVector = addMovementComponents(forwardSpeed, robotAngle, rotationSpeed, 225);
    BRVector = addMovementComponents(forwardSpeed, robotAngle, rotationSpeed, 45);
    BLVector = addMovementComponents(forwardSpeed, robotAngle, rotationSpeed, 315);
    double maxMagnitude = Math.max(Math.max(Math.max(Math.max(
    FRVector.magnitude(), 
    FLVector.magnitude()), 
    BLVector.magnitude()),
    BRVector.magnitude()),
    1.0);
    if(maxMagnitude > 1){
      //Normalize vectors, preserving proportions while reducing all below 1
      scaleVector2d(FRVector, 1. / maxMagnitude);
      scaleVector2d(FLVector, 1. / maxMagnitude);
      scaleVector2d(BRVector, 1. / maxMagnitude);
      scaleVector2d(BLVector, 1. / maxMagnitude);
    }
    mModules[0].setModule(getVectorAngle(FRVector), FRVector.magnitude());
    mModules[1].setModule(getVectorAngle(FLVector), FLVector.magnitude());
    mModules[2].setModule(getVectorAngle(BRVector), BRVector.magnitude());
    mModules[3].setModule(getVectorAngle(BLVector), BLVector.magnitude());
  }
  
  private Vector2d scaleVector2d(Vector2d v, double scalar){
    return new Vector2d(v.x * scalar, v.y * scalar);
  }
  
  private double getVectorAngle(Vector2d v){
    double angle = Math.atan(v.y / v.x) * 180 / 3.14;
    if(v.x < 0) angle += 180;
    else if(v.y < 0) angle += 360;
    return angle;
  }
  
  /**
  * Takes in two magnitudes and two angles, and combines them into a single vector
  * @param forwardMagnitude The speed at which the wheel should translate across the floor
  * @param rotation1 The angle, relative to the bot, to which the wheel should move.
  * @param rotationalMagnitude The speed at which the wheel rotates the robot
  * @param rotation2 The optimal angle for that wheel to face during rotation
  * @return
  */
  private Vector2d addMovementComponents(double forwardMagnitude, double rotation1, double rotationalMagnitude, double rotation2){
    Vector2d forwardVector = new Vector2d(forwardMagnitude * Math.cos(rotation1 * 3.14 / 180), forwardMagnitude * Math.sin(rotation1 * 3.14 / 180));
    Vector2d rotationVector = new Vector2d(rotationalMagnitude * Math.cos(rotation2 * 3.14 / 180), rotationalMagnitude * Math.sin(rotation2 * 3.14 / 180));
    return new Vector2d(forwardVector.x + rotationVector.x, forwardVector.y + rotationVector.y);
  }

  private double getGyroAngle() {
    return 0.0; // (mNavX.getAngleAdjustment() < 0) ? mNavX.getAngleAdjustment() + 360 : mNavX.getAngleAdjustment();
  }
  
  @Override
  public void onEnabledLoopStart(double timestamp) {
    synchronized (Drive.this) {
      stop();
    }
  }
  
  @Override
  public void onEnabledLoop(double timestamp) {
    synchronized (Drive.this) {
      
    }
  }
  
  @Override
  public void onEnabledLoopStop(double timestamp) {
    synchronized (Drive.this) {
      stop();
    }
  }
  
  
  @Override
  public void stop() {
    mModules[0].setModule(0, 0);
    mModules[1].setModule(0, 0);
    mModules[2].setModule(0, 0);
    mModules[3].setModule(0, 0);
  }
  
  @Override
  public void outputTelemetry() {
    SmartDashboard.putNumber("Front Right", mModules[0].getRotation());
    SmartDashboard.putNumber("Front Left", mModules[1].getRotation());
    SmartDashboard.putNumber("Back Right", mModules[2].getRotation());
    SmartDashboard.putNumber("Back Left", mModules[3].getRotation());
  }
  
  @Override
  public boolean checkSystem() {
    // TODO Auto-generated method stub
    return false;
  }

  public void zeroSensors() {
    mModules[0].zeroEncoders();
    mModules[1].zeroEncoders();
    mModules[2].zeroEncoders();
    mModules[3].zeroEncoders();
    // mNavX.zeroYaw();
  }

  public void setDriveMode(DriveMode mode) {
    mCurrentDriveMode = mode;
  }

  public void setRotationMode(RotationMode mode) {
    mCurrentRotationMode = mode;
  }
}

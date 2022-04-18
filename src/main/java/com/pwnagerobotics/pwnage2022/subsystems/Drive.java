package com.pwnagerobotics.pwnage2022.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pwnagerobotics.pwnage2022.Constants;
import com.pwnagerobotics.pwnage2022.lib.SwerveModule;
import com.pwnagerobotics.pwnage2022.subsystems.Drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
  
  private PIDController mFieldCentricRotationPID = new PIDController(Constants.kRotationkP, Constants.kRotationkI, Constants.kRotationkD);
  private PIDController mCompensationPID = new PIDController(Constants.kRotationkP, Constants.kRotationkI, Constants.kRotationkD);
  private SwerveModule[] mModules = new SwerveModule[4];
  private AHRS mNavX = new AHRS();
  private double mCurrentRobotAngle = 0.0;
  private double mCurrentGyroValue = 0.0;
  private double mOldGyroValue = 0.0;
  
  public Drive() {
    mModules[0] = new SwerveModule(Constants.kFrontRightModuleConstants);
    mModules[1] = new SwerveModule(Constants.kFrontLefttModuleConstants);
    mModules[2] = new SwerveModule(Constants.kBackRightModuleConstants);
    mModules[3] = new SwerveModule(Constants.kBackLefttModuleConstants);
    
    mFieldCentricRotationPID.enableContinuousInput(-180, 180);
    mCompensationPID.enableContinuousInput(-180, 180);
    mFieldCentricRotationPID.setTolerance(Constants.kFieldCentricRotationError);
    mCompensationPID.setTolerance(Constants.kFieldCentricRotationError);
    
    mNavX.setAngleAdjustment(-Constants.kGyroOffset);
  }
  
  public synchronized void setSwerveDrive(double throttle, double strafe, double rotationX, double rotationY) {
    double angle = Math.toDegrees(Math.atan2(strafe, throttle));
    angle = (angle >= 0) ? angle : angle + 360;
    double speed = Math.sqrt(Math.pow(Math.abs(strafe), 2) + Math.pow(Math.abs(throttle), 2));
    speed = (speed >= 1) ? 1 : speed;
    rotationX *= Constants.kSpinSlowDown;
    mCurrentGyroValue = mNavX.getAngle();
    mCurrentRobotAngle += mCurrentGyroValue - mOldGyroValue;

    if (mCurrentRotationMode == RotationMode.FEILD) {
      double wantedAngle = Math.toDegrees(Math.atan2(rotationX, rotationY));
      wantedAngle = (wantedAngle >= 0) ? wantedAngle : wantedAngle + 360;
      double distance = SwerveModule.getDistance(getGyroAngle(), wantedAngle);
      rotationX = mFieldCentricRotationPID.calculate(0, -distance);
      if (rotationX > 1) rotationX = 1;
      if (rotationX < -1) rotationX = -1;
      if (mFieldCentricRotationPID.atSetpoint()) rotationX = 0;
    }
    
    if (mCurrentDriveMode == DriveMode.FEILD) {
      angle -= getGyroAngle();
      // angle -= mCurrentRobotAngle; // TODO Test if this is better
      if (angle < 0) angle += 360;
      if (rotationX != 0) {
        angle -= Constants.kGyroErrorOffset*rotationX; // TODO * Constants.kSpinSlowDown
      }


    }
    if (rotationX == 0) { // Gyro Drift/Lag Compensation
      double distance = mCurrentGyroValue - mOldGyroValue;
      rotationX = mCompensationPID.calculate(0, distance) * (speed * Constants.kDriveSlowDown);
      if (rotationX > 1) rotationX = 1;
      if (rotationX < -1) rotationX = -1;
      if (mCompensationPID.atSetpoint()) rotationX = 0;
    }
    
    setVectorSwerveDrive(speed, -rotationX, angle);
    mOldGyroValue = mCurrentGyroValue;
  }
  
  
  public void setVectorSwerveDrive(double forwardSpeed, double rotationSpeed, double robotAngle){
    Vector2d FRVector, FLVector, BRVector, BLVector;
    FRVector = addMovementComponents(forwardSpeed, robotAngle, rotationSpeed, 315);
    FLVector = addMovementComponents(forwardSpeed, robotAngle, rotationSpeed, 225);
    BRVector = addMovementComponents(forwardSpeed, robotAngle, rotationSpeed, 45);
    BLVector = addMovementComponents(forwardSpeed, robotAngle, rotationSpeed, 135); 
    double maxMagnitude = Math.max(Math.max(Math.max(
    FRVector.magnitude(), 
    FLVector.magnitude()), 
    BLVector.magnitude()),
    BRVector.magnitude());
    if(maxMagnitude > 1){
      //Normalize vectors, preserving proportions while reducing all below 1
      scaleVector2d(FRVector, 1.0 / maxMagnitude);
      scaleVector2d(FLVector, 1.0 / maxMagnitude);
      scaleVector2d(BRVector, 1.0 / maxMagnitude);
      scaleVector2d(BLVector, 1.0 / maxMagnitude);
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
    double angle = Math.toDegrees(Math.atan(v.y / v.x));
    if (v.x == 0) angle = 0;
    if (v.x < 0) angle += 180;
    else if (v.y < 0) angle += 360;
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
    Vector2d forwardVector = new Vector2d(forwardMagnitude * Math.cos(Math.toRadians(rotation1)), forwardMagnitude * Math.sin(Math.toRadians(rotation1)));
    Vector2d rotationVector = new Vector2d(rotationalMagnitude * Math.cos(Math.toRadians(rotation2)), rotationalMagnitude * Math.sin(Math.toRadians(rotation2)));
    return new Vector2d(forwardVector.x + rotationVector.x, forwardVector.y + rotationVector.y);
  }
  
  private double getGyroAngle() {
    double currentAngle = mNavX.getAngle();
    if (currentAngle > 360) {
      currentAngle -= Math.round(currentAngle/360)*360;
    }
    else if (currentAngle < 0) {
      currentAngle += -Math.round(currentAngle/360) * 360 + 360;
      if (currentAngle > 360) currentAngle -= Math.round(currentAngle/360) * 360;
    }
    return currentAngle;
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
    SmartDashboard.putNumber("Gyro Angle", getGyroAngle());
    SmartDashboard.putNumber("Raw Gyro Angle", mNavX.getYaw());
  }
  
  @Override
  public void zeroSensors() {
    mNavX.setAngleAdjustment(-mNavX.getYaw());
  }
  
  @Override
  public boolean checkSystem() {
    return false;
  }
  
  public void setDriveMode(DriveMode mode) {
    mCurrentDriveMode = mode;
  }
  
  public void setRotationMode(RotationMode mode) {
    mCurrentRotationMode = mode;
  }
}

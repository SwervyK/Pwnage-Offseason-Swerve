package com.pwnagerobotics.pwnage2022.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pwnagerobotics.pwnage2022.Constants;
import com.pwnagerobotics.pwnage2022.humans.driver.XboxDriver;
import com.pwnagerobotics.pwnage2022.lib.SwerveModule;
import com.pwnagerobotics.pwnage2022.subsystems.Drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;

import com.team254.lib.subsystems.Subsystem;
import com.team254.lib.util.DelayedBoolean;

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

  private DelayedBoolean mGyroLagDelay = new DelayedBoolean(Constants.kGyroDelay);
  private boolean mCompensationActive = false;
  private PIDController mFieldCentricRotationPID = new PIDController(Constants.kRotationkP, Constants.kRotationkI, Constants.kRotationkD);
  private PIDController mCompensationPID = new PIDController(Constants.kRotationkP, Constants.kRotationkI, Constants.kRotationkD);
  private SwerveModule[] mModules = new SwerveModule[4];
  private AHRS mNavX = new AHRS();
  private PowerDistribution PDP = new PowerDistribution(0, ModuleType.kCTRE);
  private double mWantedAngle = 0.0; // Direction the robot should be pointing
  
  public Drive() {
    mModules[0] = new SwerveModule(Constants.kFrontRightModuleConstants);
    mModules[1] = new SwerveModule(Constants.kFrontLeftModuleConstants);
    mModules[2] = new SwerveModule(Constants.kBackRightModuleConstants);
    mModules[3] = new SwerveModule(Constants.kBackLeftModuleConstants);

    mFieldCentricRotationPID.enableContinuousInput(-180, 180);
    mFieldCentricRotationPID.setTolerance(Constants.kFieldCentricRotationError);
    mCompensationPID.enableContinuousInput(-180, 180);
    mCompensationPID.setTolerance(Constants.kGyroCompensationError);
    
    mNavX.setAngleAdjustment(-Constants.kGyroOffset);
  }
  
  public synchronized void setSwerveDrive(double throttle, double strafe, double rotationX, double rotationY) {
    //tuneRobotRotationPID();
    double wheelAngle = Math.toDegrees(Math.atan2(strafe, throttle)); // Find what angle we want to drive at
    wheelAngle = (wheelAngle >= 0) ? wheelAngle : wheelAngle + 360; // Convert from (-180 to 180) to (0 to 360) 
    double speed = Math.sqrt(Math.pow(Math.abs(strafe), 2) + Math.pow(Math.abs(throttle), 2)); // Get wanted speed of robot
    speed = XboxDriver.scaleController(SwerveModule.clamp(speed, 1, 0, false), Constants.kDriveMaxValue, Constants.kDriveMinValue);
    // Rotation
    if (mCurrentRotationMode == RotationMode.FEILD) {
      double wantedRobotAngle = Math.toDegrees(Math.atan2(rotationX, rotationY)); // Point robot in direction of controller using pid
      wantedRobotAngle = (wantedRobotAngle >= 0) ? wantedRobotAngle : wantedRobotAngle + 360; // Convert from (-180 to 180) to (0 to 360)
      double distance = SwerveModule.getDistance(getGyroAngle(), wantedRobotAngle);
      rotationX = SwerveModule.clamp(mFieldCentricRotationPID.calculate(0, distance), 1, -1, false);
      if (mFieldCentricRotationPID.atSetpoint()) rotationX = 0;
      SmartDashboard.putNumber("Wanted Robot Angle", wantedRobotAngle);
      SmartDashboard.putNumber("Distance to Angle", distance);
      SmartDashboard.putNumber("Turn Speed", rotationX);
    }
    else { // Make easier to drive
      rotationX = XboxDriver.scaleController(rotationX, Constants.kRotationMaxValue, Constants.kRotationMinValue); // Adjust controller
      mGyroLagDelay.update(Timer.getFPGATimestamp(), false);
      mFieldCentricRotationPID.reset(); //TODO test
    }
    
    // Drive
    if (mCurrentDriveMode == DriveMode.FEILD) {
      wheelAngle -= getGyroAngle(); // Field centric
      wheelAngle = SwerveModule.clamp(wheelAngle, 360, 0, true);
      if (mCurrentRotationMode != RotationMode.FEILD) {
        wheelAngle -= Constants.kGyroLag*rotationX; // Compensate for Gyro Lag
      }
    }
    // Gyro Drift/Lag Compensation
    if (rotationX == 0 && mCompensationActive) {
      double distance = mWantedAngle - getGyroAngle();
      rotationX = SwerveModule.clamp(mCompensationPID.calculate(0, distance), 1, -1, false);
      if (mCompensationPID.atSetpoint()) rotationX = 0;
      //mGyroLagDelay.update(Timer.getFPGATimestamp(), false);
      mGyroLagDelay.update(Timer.getFPGATimestamp(), false);
    }
    else {
      mCompensationActive = false;
    }

    if (!mCompensationActive) { // Adds a short delat to when we start using the Gyro to keep robot pointed in one direction
      if (rotationX == 0 && mGyroLagDelay.update(Timer.getFPGATimestamp(), true)) {
        mCompensationActive = true;
        mWantedAngle = getGyroAngle();
        mCompensationPID.reset(); //TODO test
      }
    }
    
    // if (throttle == 0 && strafe == 0 && rotationX == 0 && rotationY == 0) {
    //   // if (mModules[0].getDeltaDrive() < Constants.kDriveMinSpeed &&
    //   //     mModules[1].getDeltaDrive() < Constants.kDriveMinSpeed &&
    //   //     mModules[2].getDeltaDrive() < Constants.kDriveMinSpeed &&
    //   //     mModules[3].getDeltaDrive() < Constants.kDriveMinSpeed) {
    //   //       mModules[0].setModule(315, 0);
    //   //       mModules[1].setModule(225, 0);
    //   //       mModules[2].setModule(45, 0);
    //   //       mModules[3].setModule(135, 0);
    //   // }
    //   return;
    // }
    setVectorSwerveDrive(speed, -rotationX, wheelAngle); //TODO should it be -rotationX
  }
  
  private void setVectorSwerveDrive(double forwardSpeed, double rotationSpeed, double robotAngle) {
    Vector2d FRVector, FLVector, BRVector, BLVector;
    FRVector = addMovementComponents(forwardSpeed, robotAngle, rotationSpeed, 315);
    FLVector = addMovementComponents(forwardSpeed, robotAngle, rotationSpeed, 225);
    BRVector = addMovementComponents(forwardSpeed, robotAngle, rotationSpeed, 45);
    BLVector = addMovementComponents(forwardSpeed, robotAngle, rotationSpeed, 135); 
    // FRVector = addMovementComponents(forwardSpeed, robotAngle, rotationSpeed, getTurnAngle(Constants.kDriveWidth/2, Constants.kDriveLength/2));
    // FLVector = addMovementComponents(forwardSpeed, robotAngle, rotationSpeed, getTurnAngle(-Constants.kDriveWidth/2, Constants.kDriveLength/2));
    // BRVector = addMovementComponents(forwardSpeed, robotAngle, rotationSpeed, getTurnAngle(Constants.kDriveWidth/2, -Constants.kDriveLength/2));
    // BLVector = addMovementComponents(forwardSpeed, robotAngle, rotationSpeed, getTurnAngle(-Constants.kDriveWidth/2, -Constants.kDriveLength/2)); 
    double maxMagnitude = Math.max(Math.max(Math.max(
    FRVector.magnitude(), 
    FLVector.magnitude()), 
    BLVector.magnitude()),
    BRVector.magnitude());
    if(maxMagnitude > 1){
      //Normalize vectors, preserving proportions while reducing all below 1
      FRVector = scaleVector2d(FRVector, 1.0 / maxMagnitude);
      FLVector = scaleVector2d(FLVector, 1.0 / maxMagnitude);
      BRVector = scaleVector2d(BRVector, 1.0 / maxMagnitude);
      BLVector = scaleVector2d(BLVector, 1.0 / maxMagnitude);
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

  private double getTurnAngle(double xPos, double yPos) {
    
    return 0;
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

  private void tuneRobotRotationPID() {
    if (-1 == SmartDashboard.getNumber("kP", -1)) SmartDashboard.putNumber("kP", 0);
    if (-1 == SmartDashboard.getNumber("kI", -1)) SmartDashboard.putNumber("kI", 0);
    if (-1 == SmartDashboard.getNumber("kD", -1)) SmartDashboard.putNumber("kD", 0);
    mFieldCentricRotationPID.setPID(SmartDashboard.getNumber("kP", 0), SmartDashboard.getNumber("kI", 0), SmartDashboard.getNumber("kD", 0));
  }
  
  private double getGyroAngle() { 
    double currentAngle = mNavX.getAngle();
    if (currentAngle > 360) {
      currentAngle -= Math.round(currentAngle/360)*360; //TODO use clamp?
    }
    else if (currentAngle < 0) {
      currentAngle += -Math.round(currentAngle/360) * 360 + 360;
      if (currentAngle > 360) currentAngle -= Math.round(currentAngle/360) * 360;
    }
    return currentAngle;
  }

  public double getCurrent(int port) {
    return PDP.getCurrent(port);
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
    SmartDashboard.putNumber("Front Right Angle", mModules[0].getRotation());
    SmartDashboard.putNumber("Front Left Angle", mModules[1].getRotation());
    SmartDashboard.putNumber("Back Right Angle", mModules[2].getRotation());
    SmartDashboard.putNumber("Back Left Angle", mModules[3].getRotation());
    SmartDashboard.putNumber("Gyro Angle", getGyroAngle());
    SmartDashboard.putNumber("Raw Gyro Angle", mNavX.getYaw());
    mModules[0].outputTelemetry();
    mModules[1].outputTelemetry();
    mModules[2].outputTelemetry();
    mModules[3].outputTelemetry();
  }
  
  @Override
  public void zeroSensors() {
    mNavX.setAngleAdjustment(-mNavX.getYaw());
    mWantedAngle = 0;
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

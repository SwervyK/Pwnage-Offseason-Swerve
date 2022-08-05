package com.pwnagerobotics.pwnage2022.subsystems;

import javax.xml.crypto.dsig.keyinfo.KeyInfo;

import com.kauailabs.navx.frc.AHRS;
import com.pwnagerobotics.pwnage2022.Constants;
import com.pwnagerobotics.pwnage2022.Kinematics;
import com.pwnagerobotics.pwnage2022.humans.driver.XboxDriver;
import com.pwnagerobotics.pwnage2022.lib.SwerveModule;
import com.pwnagerobotics.pwnage2022.subsystems.Drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.subsystems.Subsystem;
import com.team254.lib.util.DelayedBoolean;

public class Drive extends Subsystem {

  public static Drive mInstance;
  public synchronized static Drive getInstance() {
    if (mInstance == null) mInstance = new Drive();
    return mInstance;
  }
  
  public enum DriveMode {
    ROBOT,
    FEILD
  }
  private DriveMode mCurrentDriveMode = DriveMode.FEILD;
  public enum RotationMode {
    ROBOT,
    FEILD
  }
  private RotationMode mCurrentRotationMode = RotationMode.ROBOT;

  private DelayedBoolean mGyroLagDelay = new DelayedBoolean(Constants.kGyroDelay);
  private boolean mCompensationActive = false;
  private PIDController mFieldCentricRotationPID = new PIDController(Constants.kRotationkP, Constants.kRotationkI, Constants.kRotationkD);
  private PIDController mCompensationPID = new PIDController(Constants.kCompensationP, Constants.kCompensationI, Constants.kCompensationD);
  private SwerveModule[] mModules = new SwerveModule[4];
  private AHRS mNavX = new AHRS();
  private PowerDistribution PDP = new PowerDistribution(0, ModuleType.kCTRE);
  private double mWantedAngle = 0.0; // Direction the robot should be pointing

  private double mLastNonZeroRobotAngle = 0;
  
  public Drive() {
    mModules[0] = new SwerveModule(Constants.kFrontRightModuleConstants);
    mModules[1] = new SwerveModule(Constants.kFrontLeftModuleConstants);
    mModules[2] = new SwerveModule(Constants.kBackRightModuleConstants);
    mModules[3] = new SwerveModule(Constants.kBackLeftModuleConstants);

    mFieldCentricRotationPID.enableContinuousInput(-180, 180);
    mFieldCentricRotationPID.setTolerance(Constants.kFieldCentricRotationError);
    mCompensationPID.enableContinuousInput(-180, 180);
    mCompensationPID.setTolerance(Constants.kCompensationError);
    
    mNavX.setAngleAdjustment(-Constants.kGyroOffset);
  }
  
  public void setSwerveDrive(double throttle, double strafe, double rotationX, double rotationY) {
    double robotAngle = Math.toDegrees(Math.atan2(strafe, throttle)); // Find what angle we want to drive at
    robotAngle = (robotAngle >= 0) ? robotAngle : robotAngle + 360; // Convert from (-180 to 180) to (0 to 360)
    double speed = Math.sqrt(Math.pow(Math.abs(strafe), 2) + Math.pow(Math.abs(throttle), 2)); // Get wanted speed of robot
    speed = XboxDriver.scaleController(SwerveModule.clamp(speed, 1, 0, false), Constants.kDriveMaxValue, Constants.kDriveMinValue);
    
    double controllerAngle = robotAngle; // TODO change, verry sloppy fix
    // Pole Snapping
    robotAngle = nearestPoleSnap(robotAngle);

    // Rotation
    if (mCurrentRotationMode == RotationMode.FEILD) {
      double wantedRobotAngle = Math.toDegrees(Math.atan2(rotationX, rotationY)); // Point robot in direction of controller using pid
      wantedRobotAngle = (wantedRobotAngle >= 0) ? wantedRobotAngle : wantedRobotAngle + 360; // Convert from (-180 to 180) to (0 to 360)
      double distance = SwerveModule.getDistance(getGyroAngle(), wantedRobotAngle);
      rotationX = SwerveModule.clamp(mFieldCentricRotationPID.calculate(0, -distance), 1, -1, false);
      if (mFieldCentricRotationPID.atSetpoint()) rotationX = 0;
      rotationX = XboxDriver.scaleController(rotationX, Constants.kRotationMaxValue, Constants.kRotationMinValue);
    }
    else { // Make easier to drive
      rotationX = XboxDriver.scaleController(rotationX, Constants.kRotationMaxValue, Constants.kRotationMinValue); // Adjust controller
      mFieldCentricRotationPID.reset(); //TODO test
    }
    
    // Drive
    if (mCurrentDriveMode == DriveMode.FEILD) {
      robotAngle -= getGyroAngle(); // Field centric
      robotAngle = SwerveModule.clamp(robotAngle, 360, 0, true);
      if (mCurrentRotationMode != RotationMode.FEILD) {
        robotAngle -= Constants.kGyroLag*rotationX; // Compensate for Gyro Lag // TODO Gyro
      }
    }
    // Gyro Drift/Lag Compensation
    if (rotationX == 0 && mCompensationActive) {
      double distance = mWantedAngle - getGyroAngle();
      rotationX = SwerveModule.clamp(mCompensationPID.calculate(0, distance), 1, -1, false);
      rotationX = XboxDriver.scaleController(rotationX, Constants.kRotationMaxValue, Constants.kRotationMinValue); // So even when PID returns small values the robot still moves
      if (mCompensationPID.atSetpoint() /*|| Math.abs(distance) > Constants.kStartCompensation*/) rotationX = 0; // TODO add back?
      mGyroLagDelay.update(Timer.getFPGATimestamp(), false);
    }
    else {
      mCompensationActive = false;
    }

    // if (/* Motors move and we dont */) {
    //   mGyroLagDelay.update(Timer.getFPGATimestamp(), false);
    // }
    
    if (!mCompensationActive) { // Adds a short delat to when we start using the Gyro to keep robot pointed in one direction
      if (rotationX == 0 && mGyroLagDelay.update(Timer.getFPGATimestamp(), true)) { // TODO Gyro
        mCompensationActive = true;
        mWantedAngle = getGyroAngle();
        //mCompensationPID.reset(); //TODO test
      }
    }
    
    // When robot is moving slow enough set all module angles to *45 to make us harder to push
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
    if (speed == 0 && controllerAngle == 0) { // Dont set module direction to 0 if not moving
      robotAngle = mLastNonZeroRobotAngle;
    }
    setVectorSwerveDrive(speed, -rotationX, robotAngle);
    if (controllerAngle != 0)
      mLastNonZeroRobotAngle = controllerAngle;
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
    // When robot is not moving set angle to robotAngle (otherwise if rotationSpeed and driveSpeed are 0 angle is 0)
    if (forwardSpeed == 0 && rotationSpeed == 0)  {
      mModules[0].setModule(robotAngle, 0);
      mModules[1].setModule(robotAngle, 0);
      mModules[2].setModule(robotAngle, 0);
      mModules[3].setModule(robotAngle, 0);
    }
    else {
      mModules[0].setModule(getVectorAngle(FRVector), FRVector.magnitude());
      mModules[1].setModule(getVectorAngle(FLVector), FLVector.magnitude());
      mModules[2].setModule(getVectorAngle(BRVector), BRVector.magnitude());
      mModules[3].setModule(getVectorAngle(BLVector), BLVector.magnitude());
    }
  }
  
  private Vector2d scaleVector2d(Vector2d v, double scalar){
    return new Vector2d(v.x * scalar, v.y * scalar);
  }
  
  // Get angle out of vector
  private double getVectorAngle(Vector2d v) {
    double angle = Math.toDegrees(Math.atan(v.y / v.x));
    if (v.x == 0) angle = 0;
    if (v.x < 0) angle += 180;
    else if (v.y < 0) angle += 360;
    return angle;
  }

  private double nearestPoleSnap(double angle) {
    double poleSin = 0.0;
    double poleCos = 0.0;
    double sin = Math.sin(Math.toRadians(angle));
    double cos = Math.cos(Math.toRadians(angle));
    if (Math.abs(cos) > Math.abs(sin)) {
      poleCos = Math.signum(cos);
      poleSin = 0.0;
    } 
    else {
      poleCos = 0.0;
      poleSin = Math.signum(sin);
    }
    double pole = Math.toDegrees(Math.atan2(poleSin, poleCos));
    pole = (pole >= 0) ? pole : pole + 360; // Convert from -180 to 180 to 0 to 360
    if (Math.abs(pole - angle) <= Constants.kPoleSnappingThreshold) {
      // change angle from 359 to 1 so pole snapping works TODO make better
      if (angle > 270 + Constants.kPoleSnappingThreshold) angle = Math.abs(angle-360); 
      return Math.toDegrees(Math.atan2(poleSin, poleCos));
    }
    else 
      return angle;
  }

  // Get module spin angles using x and y position
  // EX: on a square robot everything is 45 degrees
  private double getTurnAngle(double xPos, double yPos) { // TODO finish
    
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
    mCompensationPID.setPID(SmartDashboard.getNumber("kP", 0), SmartDashboard.getNumber("kI", 0), SmartDashboard.getNumber("kD", 0));
  }
  
  // Gyro angle does not loop around so it can be 360> and <-360 so this wraps it around to -360:360
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

  public void PoseToDrive(Pose2d velocity) {
    Translation2d[] wheelVelocities = Kinematics.inverseKinematics(velocity);

    mModules[0].setModule(wheelVelocities[0].direction().getDegrees(), Math.sqrt(wheelVelocities[0].norm2()));
    mModules[1].setModule(wheelVelocities[1].direction().getDegrees(), Math.sqrt(wheelVelocities[1].norm2()));
    mModules[2].setModule(wheelVelocities[2].direction().getDegrees(), Math.sqrt(wheelVelocities[2].norm2()));
    mModules[3].setModule(wheelVelocities[3].direction().getDegrees(), Math.sqrt(wheelVelocities[3].norm2()));
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

  public double getCurrent(int port) {
    return PDP.getCurrent(port);
  }

  public void setModule(int module, double angle, double speed) {
    mModules[module].setModule(angle, speed);
  }

  public Rotation2d getRobotAngle() {
    return new Rotation2d(Math.toRadians(getGyroAngle()), false);
  }

  public Translation2d getModuleState(int module) {
    return new Translation2d(mModules[module].getDriveVelocity()*Math.cos(Math.toRadians(mModules[module].getRotation())), mModules[module].getDriveVelocity()*Math.sin(Math.toRadians(mModules[module].getRotation())));
  }
}

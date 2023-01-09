package com.pwnagerobotics.pwnage2022.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pwnagerobotics.pwnage2022.Constants;
import com.pwnagerobotics.pwnage2022.Kinematics;
import com.pwnagerobotics.pwnage2022.humans.driver.XboxDriver;
import com.pwnagerobotics.pwnage2022.lib.SwerveDriveHelper;
import com.pwnagerobotics.pwnage2022.lib.SwerveModule;
import com.pwnagerobotics.pwnage2022.subsystems.Drive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.subsystems.Subsystem;
import com.team254.lib.util.SynchronousPIDF;

public class Drive extends Subsystem {
  
  public boolean DEBUG_MODE = true;
  public boolean TUNING = true;
  
  public static Drive mInstance;
  public synchronized static Drive getInstance() {
    if (mInstance == null) mInstance = new Drive();
    return mInstance;
  }
  
  public enum DriveMode {
    ROBOT,
    FIELD
  }
  private DriveMode mCurrentDriveMode = DriveMode.FIELD;
  public enum RotationMode {
    ROBOT,
    FIELD
  }
  private RotationMode mCurrentRotationMode = RotationMode.ROBOT;
  
  private SynchronousPIDF mFieldCentricRotationPID = new SynchronousPIDF(Constants.kRotationP, Constants.kRotationI, Constants.kRotationD);
  private SynchronousPIDF mCompensationPID = new SynchronousPIDF(Constants.kCompensationP, Constants.kCompensationI, Constants.kCompensationD);
  private double mCompensationPIDTolerance = Constants.kCompensationErrorLow;
  private SwerveModule[] mModules = new SwerveModule[4];
  private AHRS mNavX = new AHRS();
  private double mWantedRobotAngleRadians = 0.0; // Direction the robot should be pointing
  private double mLastNonZeroRobotAngle = 0;
  private double mLastGyro = 0;
  private double mRobotCenterDisplacementX = 0;
  private double mRobotCenterDisplacementY = 0;
  
  public Drive() {
    mPeriodicIO = new PeriodicIO();
    mModules[0] = new SwerveModule(Constants.kFrontRightModuleConstants);
    mModules[1] = new SwerveModule(Constants.kFrontLeftModuleConstants);
    mModules[2] = new SwerveModule(Constants.kBackRightModuleConstants);
    mModules[3] = new SwerveModule(Constants.kBackLeftModuleConstants);
    
    mFieldCentricRotationPID.setInputRange(-180, 180);
    mFieldCentricRotationPID.setOutputRange(-1, 1);
    mCompensationPID.setInputRange(-180, 180);
    mCompensationPID.setOutputRange(-1, 1);
    
    mNavX.setAngleAdjustment(-Constants.kGyroOffset);
  }
  
  public void setKinematicsDrive(final double throttle, final double strafe, final double rotationX, final double rotationY) {
    if (TUNING) tuneRobotRotationPID();
    // Apply effects to inputs
    double[] effects = SwerveDriveHelper.applyControlEffects(throttle, strafe, rotationX, rotationY, mCurrentDriveMode, mPeriodicIO.gyro_angle_radians);
    double direction = effects[0];
    double magnitude = effects[1];
    double rotation = rotationX;

    // Rotation
    if (mCurrentRotationMode == RotationMode.FIELD) { // Point robot in direction of controller using pid
      double wantedRobotAngle = Math.atan2(rotationX, rotationY);
      double distance = SwerveDriveHelper.getAngularDistance(mPeriodicIO.gyro_angle_radians, wantedRobotAngle, Math.PI*2);
      mFieldCentricRotationPID.setSetpoint(distance);
      rotation = SwerveDriveHelper.clamp(mFieldCentricRotationPID.calculate(0), 1, -1, false);
      if (mFieldCentricRotationPID.onTarget(Constants.kFieldCentricRotationError)) rotation = 0;
      if (DEBUG_MODE) SmartDashboard.putNumber("Field Centric Rot Rot", rotation);
      if (DEBUG_MODE) SmartDashboard.putNumber("Field Centric Rot Distance", distance);
    }
    else { // Make easier to drive
      rotation = XboxDriver.scaleController(rotationX, Constants.kRotationMaxValue, Constants.kRotationMinValue); // Adjust controller
      mFieldCentricRotationPID.reset();
    }
    
    // Drive Compensation
    if (rotationX == 0 && Math.abs(gyroDelta()) < Constants.kMinGyroDelta) {
      double distance = SwerveDriveHelper.getAngularDistance(mPeriodicIO.gyro_angle_radians, mWantedRobotAngleRadians, Math.PI*2);
      mCompensationPID.setSetpoint(distance);
      rotation = SwerveDriveHelper.clamp(mCompensationPID.calculate(0), 1, -1, false);
      if (mCompensationPID.onTarget(mCompensationPIDTolerance)) {
        rotation = 0;
        mCompensationPIDTolerance = Constants.kCompensationErrorHigh;
      }
      if (Math.abs(distance) > Constants.kCompensationErrorHigh) {
        mCompensationPIDTolerance = Constants.kCompensationErrorLow;
      }
      if (DEBUG_MODE) SmartDashboard.putBoolean("Compensation Active", true);
      if (DEBUG_MODE) SmartDashboard.putNumber("Compensation Rot", rotationX);
      if (DEBUG_MODE) SmartDashboard.putNumber("Compensation Distance", distance);
    }
    else {
      mCompensationPID.reset();
      mCompensationPIDTolerance = Constants.kCompensationErrorLow;
      mWantedRobotAngleRadians = mPeriodicIO.gyro_angle_radians;
      if (DEBUG_MODE) SmartDashboard.putBoolean("Compensation Active", false);
    }
    
    // if (/* Motors move and we dont */) {
    //   mGyroLagDelay.update(Timer.getFPGATimestamp(), false);
    // }
      
    if (magnitude == 0) { // Dont set module direction to 0 if not moving
    direction = mLastNonZeroRobotAngle;
    }
    if (magnitude != 0) {
      mLastNonZeroRobotAngle = direction;
    }

    // Forward, Strafe, Rotation
    double[] controllerInputs = SwerveDriveHelper.convertControlEffects(magnitude, direction, rotation);
    Object[][] module = Kinematics.inverseKinematics(controllerInputs, mPeriodicIO.gyro_angle_radians, mCurrentDriveMode == DriveMode.FIELD, new double[]{mRobotCenterDisplacementX, mRobotCenterDisplacementY});
    
    for (int i = 0; i < module[0].length; i++) {
      mPeriodicIO.module_magnitudes[i] = (double)module[0][i]; 
      double moduleRotation = ((Rotation2d)module[1][i]).getDegrees();
      if (moduleRotation < 0) moduleRotation += 360;
      mPeriodicIO.module_angles[i] = moduleRotation;
    }
    
    mLastGyro = mPeriodicIO.gyro_angle_radians;
  }
    
  public void jukeMove(boolean jukeRight, boolean jukeLeft) {
    double[] robotCenter = SwerveDriveHelper.jukeMove(jukeRight, jukeLeft, mLastNonZeroRobotAngle - mPeriodicIO.gyro_angle_radians, mRobotCenterDisplacementX, mRobotCenterDisplacementY);
    mRobotCenterDisplacementX = robotCenter[0];
    mRobotCenterDisplacementY = robotCenter[1];
  }
  
  private void tuneRobotRotationPID() {
    if (-1 == SmartDashboard.getNumber("kP", -1)) SmartDashboard.putNumber("kP", 0);
    if (-1 == SmartDashboard.getNumber("kI", -1)) SmartDashboard.putNumber("kI", 0);
    if (-1 == SmartDashboard.getNumber("kD", -1)) SmartDashboard.putNumber("kD", 0);
    mFieldCentricRotationPID.setPID(SmartDashboard.getNumber("kP", 0), SmartDashboard.getNumber("kI", 0), SmartDashboard.getNumber("kD", 0));
  }
  
  private PeriodicIO mPeriodicIO;
  public static class PeriodicIO {
    // Inputs
    public double[] drive_distances = new double[4];
    public double[] drive_deltas = new double[4];
    public double[] drive_velocities = new double[4];
    
    public double[] rotation_angles = new double[4];
    public double[] rotation_deltas = new double[4];
    public double[] rotation_velocities = new double[4];
    
    public double gyro_angle_radians;
    
    // Outputs
    public double[] module_angles = new double[4];
    public double[] module_magnitudes = new double[4];
  }
  
  @Override
  public void readPeriodicInputs() {
    for (int i = 0; i < mModules.length; i++) {
      mPeriodicIO.drive_distances[i] = mModules[i].getDrive();
      mPeriodicIO.drive_deltas[i] = mModules[i].getDriveDelta();
      mPeriodicIO.drive_velocities[i] = mModules[i].getDriveVelocity();
      
      mPeriodicIO.rotation_angles[i] = mModules[i].getRotationDegrees();
      mPeriodicIO.rotation_deltas[i] = mModules[i].getRotationDelta();
    }
    
    mPeriodicIO.gyro_angle_radians = SwerveDriveHelper.clamp(Math.toRadians(mNavX.getAngle()), Math.PI*2, 0, true);
  }
  
  @Override
  public void writePeriodicOutputs() {
    for (int i = 0; i < mModules.length; i++) mModules[i].setModuleDegrees(mPeriodicIO.module_angles[i], mPeriodicIO.module_magnitudes[i]);
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
    for (int i = 0; i < mModules.length; i++) {
      mPeriodicIO.module_angles[i] = 0; 
      mPeriodicIO.module_magnitudes[i] = 0;
      mModules[i].stop();
    }
    mCompensationPID.reset();
    mFieldCentricRotationPID.reset();
  }
  
  @Override
  public void outputTelemetry() {
    SmartDashboard.putNumber("Front Right Angle", mPeriodicIO.rotation_angles[0]);
    SmartDashboard.putNumber("Front Left Angle", mPeriodicIO.rotation_angles[1]);
    SmartDashboard.putNumber("Back Right Angle", mPeriodicIO.rotation_angles[2]);
    SmartDashboard.putNumber("Back Left Angle", mPeriodicIO.rotation_angles[3]);
    SmartDashboard.putNumber("Gyro Angle", mPeriodicIO.gyro_angle_radians);
    SmartDashboard.putNumber("Raw Gyro Angle", mNavX.getYaw());
    for (SwerveModule m : mModules) m.outputTelemetry();
  }
  
  @Override
  public void zeroSensors() {
    mNavX.setAngleAdjustment(-mNavX.getYaw());
    mWantedRobotAngleRadians = 0;
  }
  
  @Override
  public boolean checkSystem() {
    return false;
  }
  
  /**
  * Converts Pose2d (angle, magnitude, and rotation) to swerve module angles and magnitudes
  * @param velocity Velocity to convert
  */
  public void PoseToDrive(Pose2d velocity) {
    Translation2d[] wheelVelocities = Kinematics.inverseKinematics(velocity);
    
    for (int i = 0; i < mModules.length; i++) {
      mPeriodicIO.module_angles[i] = wheelVelocities[i].direction().getDegrees();
      mPeriodicIO.module_magnitudes[i] = Math.sqrt(wheelVelocities[i].norm2());
    }
  }
  
  public void setDriveMode(DriveMode mode) {
    mCurrentDriveMode = mode;
  }
  
  public void setRotationMode(RotationMode mode) {
    mCurrentRotationMode = mode;
  }
  
  public void setModuleDegrees(int module, double angle, double magnitude) {
    mPeriodicIO.module_angles[module] = angle;
    mPeriodicIO.module_magnitudes[module] = magnitude;
  }
  
  public Rotation2d getRobotAngle() {
    return new Rotation2d(mPeriodicIO.gyro_angle_radians, false);
  }
  
  public Translation2d getModuleState(int module) {
    return new Translation2d(mPeriodicIO.drive_velocities[module]*Math.cos(Math.toRadians(mPeriodicIO.rotation_angles[module])), mPeriodicIO.drive_velocities[module]*Math.sin(Math.toRadians(mPeriodicIO.rotation_angles[module])));
  }
  
  public double[] getModuleVelocities() {
    double[] velocities = new double[4];
    for (int i = 0; i < velocities.length; i++) {
      velocities[i] = mModules[i].getDriveVelocity();
    }
    return velocities;
  }
  
  public Rotation2d[] getModuleRotations() {
    Rotation2d[] rotations = new Rotation2d[4];
    for (int i = 0; i < rotations.length; i++) {
      rotations[i] = new Rotation2d(mModules[i].getRotationDegrees(), false);
    }
    return rotations;
  }
  
  public double gyroDelta() {
    return Math.abs(mLastGyro - mPeriodicIO.gyro_angle);
  }
  
  public double getGyroRadians() {
    return mPeriodicIO.gyro_angle_radians;
  }
}
  
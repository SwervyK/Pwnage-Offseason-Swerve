package com.pwnagerobotics.pwnage2022.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pwnagerobotics.pwnage2022.Constants;
import com.pwnagerobotics.pwnage2022.Kinematics;
import com.pwnagerobotics.pwnage2022.humans.driver.XboxDriver;
import com.pwnagerobotics.pwnage2022.lib.SwerveModule;
import com.pwnagerobotics.pwnage2022.lib.Util;
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

  private final boolean DEBUG_MODE = false;

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

  private DelayedBoolean mGyroLagDelay = new DelayedBoolean(Constants.kGyroDelay); // TODO change?
  private boolean mCompensationActive = false;
  private PIDController mFieldCentricRotationPID = new PIDController(Constants.kRotationkP, Constants.kRotationkI, Constants.kRotationkD);
  private PIDController mCompensationPID = new PIDController(Constants.kCompensationP, Constants.kCompensationI, Constants.kCompensationD);
  private SwerveModule[] mModules = new SwerveModule[4];
  private AHRS mNavX = new AHRS();
  private PowerDistribution PDP = new PowerDistribution(0, ModuleType.kRev);
  private double mWantedAngle = 0.0; // Direction the robot should be pointing
  private double mLastNonZeroRobotAngle = 0;
  private double mLastGyro;
  
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
  
  /**
  * Sets swerve drivetrain
  * @param throttle Forward/Backward movement (-1 to 1)
  * @param strafe Right/Left movement (-1 to 1)
  * @param rotationX If robot centric rotation: rotation speed (-1 to 1), If field centric rotation: X value of angle (-1 to 1)
  * @param rotationY If field centric rotation: Y value of angle (-1 to 1)
  */
  public void setSwerveDrive(double throttle, double strafe, double rotationX, double rotationY) {
    if (DEBUG_MODE) tuneRobotRotationPID();
    double direction = Math.toDegrees(Math.atan2(strafe, throttle)); // Find what angle we want to drive at
    if (direction < 0) direction += 360; // Convert from (-180 to 180) to (0 to 360)
    double magnitude = Math.hypot(Math.abs(strafe), Math.abs(throttle)); // Get wanted speed of robot
    magnitude = XboxDriver.scaleController(Util.clamp(magnitude, 1, 0, false), Constants.kDriveMaxValue, Constants.kDriveMinValue); // TODO dont need to do this twice
    
    double controllerAngle = direction;
    // Pole Snapping
    direction = nearestPoleSnap(direction);

    // Rotation
    if (mCurrentRotationMode == RotationMode.FEILD) { // Point robot in direction of controller using pid
      double wantedRobotAngle = Math.toDegrees(Math.atan2(rotationX, rotationY));
      if (wantedRobotAngle < 0) wantedRobotAngle += 360;
      double distance = Util.getDistance(mPeriodicIO.gyro_angle, wantedRobotAngle);
      rotationX = Util.clamp(mFieldCentricRotationPID.calculate(0, -distance), 1, -1, false);
      rotationX = XboxDriver.scaleController(rotationX, Constants.kRotationMaxValue, Constants.kRotationMinValue);
      if (mFieldCentricRotationPID.atSetpoint()) rotationX = 0;
    }
    else { // Make easier to drive
      rotationX = XboxDriver.scaleController(rotationX, Constants.kRotationMaxValue, Constants.kRotationMinValue); // Adjust controller
      mFieldCentricRotationPID.reset();
    }
    
    // Drive
    if (mCurrentDriveMode == DriveMode.FEILD) {
      direction -= mPeriodicIO.gyro_angle; // Field centric
      direction = Util.clamp(direction, 360, 0, true);
      if (mCurrentRotationMode != RotationMode.FEILD) {
        direction -= Constants.kGyroLag*rotationX; // Compensate for Gyro Lag // TODO Gyro
      }
    }
    // Robot Drift/Lag Compensation
    if (gyroDelta() < Constants.kMinGyroDelta && mCompensationActive) {
      double distance = mPeriodicIO.gyro_angle - mWantedAngle;
      rotationX = Util.clamp(mCompensationPID.calculate(0, -distance), 1, -1, false);
      rotationX = XboxDriver.scaleController(rotationX, Constants.kRotationMaxValue, Constants.kRotationMinValue);
      if (mCompensationPID.atSetpoint()) rotationX = 0;
      mGyroLagDelay.update(Timer.getFPGATimestamp(), false);
    }
    else {
      mCompensationActive = false;
    }

    // if (/* Motors move and we dont */) {
    //   mGyroLagDelay.update(Timer.getFPGATimestamp(), false);
    // }
    
    if (!mCompensationActive) { // Adds a short delay to when we start using the gyro to keep robot pointed in one direction
      if (gyroDelta() < Constants.kMinGyroDelta && mGyroLagDelay.update(Timer.getFPGATimestamp(), true)) {
        mCompensationActive = true;
        mWantedAngle = mPeriodicIO.gyro_angle;
        mCompensationPID.reset();
      }
    }
    
    // When robot is moving slow enough set all module angles to *45 to make us harder to push
    // if (throttle == 0 && strafe == 0 && rotationX == 0 && rotationY == 0) {
    //   if (mModules[0].getDeltaDrive() < Constants.kDriveMinSpeed &&
    //       mModules[1].getDeltaDrive() < Constants.kDriveMinSpeed &&
    //       mModules[2].getDeltaDrive() < Constants.kDriveMinSpeed &&
    //       mModules[3].getDeltaDrive() < Constants.kDriveMinSpeed) {
    //         for (int i = 0; i < mModules.length; i++) {
    //           mModules[i].setModule(
    //             getTurnAngle(i%2==0?Constants.kDriveWidth/2:-Constants.kDriveWidth/2, i<2?Constants.kDriveLength/2:-Constants.kDriveLength),
    //             0);
    //         }
    //   }
    //   return;
    // }

    if (controllerAngle == 0) { // Dont set module direction to 0 if not moving
      direction = mLastNonZeroRobotAngle;
    }
    setVectorSwerveDrive(magnitude, -rotationX, direction);
    if (controllerAngle != 0)
      mLastNonZeroRobotAngle = controllerAngle;
      mLastGyro = mPeriodicIO.gyro_angle;
  }
  
  private void setVectorSwerveDrive(double forwardSpeed, double rotationSpeed, double robotAngle) {
    Vector2d[] vectors = new Vector2d[4];
    // FRVector = addMovementComponents(forwardSpeed, robotAngle, rotationSpeed, 315);
    // FLVector = addMovementComponents(forwardSpeed, robotAngle, rotationSpeed, 225);
    // BRVector = addMovementComponents(forwardSpeed, robotAngle, rotationSpeed, 45);
    // BLVector = addMovementComponents(forwardSpeed, robotAngle, rotationSpeed, 135); 
    for (int i = 0; i < mModules.length; i++) {
      vectors[i] = addMovementComponents(forwardSpeed, robotAngle, rotationSpeed, 
      getTurnAngle(i%2==0?Constants.kDriveWidth/2:-Constants.kDriveWidth/2, i<2?Constants.kDriveLength/2:-Constants.kDriveLength)
      );
    }
    double maxMagnitude = Math.max(Math.max(Math.max(
    vectors[0].magnitude(), 
    vectors[1].magnitude()), 
    vectors[2].magnitude()),
    vectors[3].magnitude());
    if(maxMagnitude > 1){
      //Normalize vectors, preserving proportions while reducing all below 1
      for (Vector2d v : vectors) v = scaleVector2d(v, 1.0 / maxMagnitude);
    }
    // When robot is not moving set angle to robotAngle (if this is not done and rotationSpeed and driveSpeed are 0, angle is 0)
    if (forwardSpeed == 0 && rotationSpeed == 0)  {
      for (int i = 0; i < mModules.length; i++) {
        mPeriodicIO.module_angles[i] = robotAngle;
        mPeriodicIO.module_magnitudes[i] = 0;
      }
    }
    else {
      for (int i = 0; i < mModules.length; i++) {
        mPeriodicIO.module_angles[i] = getVectorAngle(vectors[i]);
        mPeriodicIO.module_magnitudes[i] = vectors[i].magnitude();
      }
    }
  }
  
  private Vector2d scaleVector2d(Vector2d v, double scalar) {
    return new Vector2d(v.x * scalar, v.y * scalar);
  }
  
  // Get angle of a vector
  private double getVectorAngle(Vector2d v) {
    double angle = Math.toDegrees(Math.atan(v.y / v.x));
    if (v.x == 0) angle = 0;
    if (v.x < 0) angle += 180;
    else if (v.y < 0) angle += 360;
    return angle;
  }

  // Get nearest pole of an angle (N S E W)
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
    if (pole < 0) pole += 360;
    if (Math.abs(pole - angle) <= Constants.kPoleSnappingThreshold) {
      // change angle from 359 to 1 so pole snapping works
      if (angle > 270 + Constants.kPoleSnappingThreshold) angle = Math.abs(angle-360); 
      return Math.toDegrees(Math.atan2(poleSin, poleCos));
    }
    else 
      return angle;
  }

  // Get module rotation angles using x and y position relative to center of robot
  // EX: on a square robot everything is 45 degrees
  private double getTurnAngle(double xPos, double yPos) {
    double theta = Math.atan2(yPos, xPos);
    theta = theta>=0?theta:theta+360;
    theta += yPos>0?180:0 + Math.signum(xPos)==Math.signum(yPos)?90:0;
    return theta;
  }
  
  /**
  * Takes in two magnitudes and two angles, and combines them into a single vector
  * @param forwardMagnitude The speed at which the wheel should translate across the floor
  * @param rotation1 The angle, relative to the bot, to which the wheel should move.
  * @param rotationalMagnitude The speed at which the wheel rotates the robot
  * @param rotation2 The optimal angle for that wheel to face during rotation
  * @return Result of the vector combination
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

  private PeriodicIO mPeriodicIO;
  public static class PeriodicIO {
    // Inputs
    public double[] drive_distances = new double[4];
    public double[] drive_deltas = new double[4];
    public double[] drive_velocitys = new double[4];

    public double[] rotation_angles = new double[4];
    public double[] rotation_deltas = new double[4];
    public double[] rotation_velocitys = new double[4];

    public double gyro_angle;

    // Outputs
    public double[] module_angles = new double[4];
    public double[] module_magnitudes = new double[4];
  }

  @Override
  public void readPeriodicInputs() {
    for (int i = 0; i < mModules.length; i++) {
      mPeriodicIO.drive_distances[i] = mModules[i].getDrive();
      mPeriodicIO.drive_deltas[i] = mModules[i].getDriveDelta();
      mPeriodicIO.drive_velocitys[i] = mModules[i].getDriveVelocity();

      mPeriodicIO.rotation_angles[i] = mModules[i].getRotation();
      mPeriodicIO.rotation_deltas[i] = mModules[i].getRotationDelta();
      mPeriodicIO.rotation_velocitys[i] = mModules[i].getRotationVelocity();
    }

    mPeriodicIO.gyro_angle = Util.clamp(mNavX.getAngle(), 360, 0, true);
  }

  @Override
  public void writePeriodicOutputs() {
    for (int i = 0; i < mModules.length; i++) mModules[i].setModule(mPeriodicIO.module_angles[i], mPeriodicIO.module_magnitudes[i]);
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
    for (SwerveModule m : mModules) m.setModule(0, 0);
  }
  
  @Override
  public void outputTelemetry() {
    SmartDashboard.putNumber("Front Right Angle", mPeriodicIO.rotation_angles[0]);
    SmartDashboard.putNumber("Front Left Angle", mPeriodicIO.rotation_angles[1]);
    SmartDashboard.putNumber("Back Right Angle", mPeriodicIO.rotation_angles[2]);
    SmartDashboard.putNumber("Back Left Angle", mPeriodicIO.rotation_angles[3]);
    SmartDashboard.putNumber("Gyro Angle", mPeriodicIO.gyro_angle);
    SmartDashboard.putNumber("Raw Gyro Angle", mNavX.getYaw());
    for (SwerveModule m : mModules) m.outputTelemetry();
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

  public double getCurrent(int port) {
    return PDP.getCurrent(port);
  }

  public void setModule(int module, double angle, double magnitude) {
    mPeriodicIO.module_angles[module] = angle;
    mPeriodicIO.module_magnitudes[module] = magnitude;
  }

  public Rotation2d getRobotAngle() {
    return new Rotation2d(Math.toRadians(mPeriodicIO.gyro_angle), false);
  }

  public Translation2d getModuleState(int module) {
    return new Translation2d(mPeriodicIO.drive_velocitys[module]*Math.cos(Math.toRadians(mPeriodicIO.rotation_angles[module])), mPeriodicIO.drive_velocitys[module]*Math.sin(Math.toRadians(mPeriodicIO.rotation_angles[module])));
  }

  public double gyroDelta() {
    return mLastGyro - mPeriodicIO.gyro_angle;
  }
}

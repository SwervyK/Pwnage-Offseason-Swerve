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
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.subsystems.Subsystem;

public class Drive extends Subsystem {

  public boolean DEBUG_MODE = true;
  public boolean TUNING = false;

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

  private PIDController mFieldCentricRotationPID = new PIDController(Constants.kRotationP, Constants.kRotationI, Constants.kRotationD);
  private PIDController mCompensationPID = new PIDController(Constants.kCompensationP, Constants.kCompensationI, Constants.kCompensationD);
  private SwerveModule[] mModules = new SwerveModule[4];
  private AHRS mNavX = new AHRS();
  private PowerDistribution PDP = new PowerDistribution(0, ModuleType.kCTRE);
  private double mWantedAngle = 0.0; // Direction the robot should be pointing
  private double mLastNonZeroRobotAngle = 0;
  private double mLastGyro;
  private double mRobotCenterX = 0;
  private double mRobotCenterY = 0;
  
  public Drive() {
    mPeriodicIO = new PeriodicIO();
    mModules[0] = new SwerveModule(Constants.kFrontRightModuleConstants);
    mModules[1] = new SwerveModule(Constants.kFrontLeftModuleConstants);
    mModules[2] = new SwerveModule(Constants.kBackRightModuleConstants);
    mModules[3] = new SwerveModule(Constants.kBackLeftModuleConstants);

    mFieldCentricRotationPID.setTolerance(Constants.kFieldCentricRotationError);
    mCompensationPID.setTolerance(Constants.kCompensationErrorLow);
    
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
    if (TUNING) tuneRobotRotationPID();
    double direction = Math.toDegrees(Math.atan2(strafe, throttle)); // Find what angle we want to drive at
    if (direction < 0) direction += 360; // Convert from (-180 to 180) to (0 to 360)
    double magnitude = Math.hypot(Math.abs(strafe), Math.abs(throttle)); // Get wanted speed of robot
    magnitude = XboxDriver.scaleController(Util.clamp(magnitude, 1, 0, false), Constants.kDriveMaxValue, Constants.kDriveMinValue);
    
    double controllerAngle = direction;
    if (DEBUG_MODE) SmartDashboard.putNumber("Controller Dir", direction);
    if (DEBUG_MODE) SmartDashboard.putNumber("Controller Mag", magnitude);
    // Pole Snapping
    if (magnitude > Constants.kPoleSnappingThreshold) direction = nearestPoleSnap(direction, Constants.kPoleSnappingAngle);

    // Rotation
    if (mCurrentRotationMode == RotationMode.FIELD) { // Point robot in direction of controller using pid
      double wantedRobotAngle = Math.toDegrees(Math.atan2(rotationX, rotationY));
      if (wantedRobotAngle < 0) wantedRobotAngle += 360;
      double distance = Util.getDistance(mPeriodicIO.gyro_angle, wantedRobotAngle);
      rotationX = Util.clamp(mFieldCentricRotationPID.calculate(0, distance), 1, -1, false);
      if (mFieldCentricRotationPID.atSetpoint()) rotationX = 0;
      if (DEBUG_MODE) SmartDashboard.putNumber("Field Centric Rot", rotationX);
    }
    else { // Make easier to drive
      rotationX = XboxDriver.scaleController(rotationX, Constants.kRotationMaxValue, Constants.kRotationMinValue); // Adjust controller
      mFieldCentricRotationPID.reset();
    }
    
    // Drive
    if (mCurrentDriveMode == DriveMode.FIELD) {
      direction -= mPeriodicIO.gyro_angle; // Field centric
      direction = Util.clamp(direction, 360, 0, true);
      if (mCurrentRotationMode != RotationMode.FIELD) {
        direction -= Constants.kGyroLag*rotationX; // Compensate for Lag? // TODO Gyro
      }
    }

    // Drive Compensation
    if (rotationX == 0 && Math.abs(gyroDelta()) < Constants.kMinGyroDelta) {
      double distance = Util.getDistance(mPeriodicIO.gyro_angle, mWantedAngle);
      rotationX = Util.clamp(mCompensationPID.calculate(0, distance), 1, -1, false);
      if (mCompensationPID.atSetpoint()) {
        rotationX = 0;
        mCompensationPID.setTolerance(Constants.kCompensationErrorHigh);
      }
      if (Math.abs(distance) > Constants.kCompensationErrorHigh) {
        mCompensationPID.setTolerance(Constants.kCompensationErrorLow);
      }
      if (DEBUG_MODE) SmartDashboard.putBoolean("Compensation Active", true);
      if (DEBUG_MODE) SmartDashboard.putNumber("Compensation Rot", rotationX);
    }
    else {
      mCompensationPID.reset();
      mCompensationPID.setTolerance(Constants.kCompensationErrorLow);
      mWantedAngle = mPeriodicIO.gyro_angle;
      if (DEBUG_MODE) SmartDashboard.putBoolean("Compensation Active", false);

    }

    // if (/* Motors move and we dont */) {
    //   mGyroLagDelay.update(Timer.getFPGATimestamp(), false);
    // }

    // // When robot is moving slow enough set all module angles to *45 to make us harder to push
    // if (throttle == 0 && strafe == 0 && rotationX == 0 && rotationY == 0) {
    //     if (mModules[0].getDeltaDrive() < Constants.kDriveMinSpeed &&
    //         mModules[1].getDeltaDrive() < Constants.kDriveMinSpeed &&
    //         mModules[2].getDeltaDrive() < Constants.kDriveMinSpeed &&
    //         mModules[3].getDeltaDrive() < Constants.kDriveMinSpeed) {
    //         for (int i = 0; i < mModules.length; i++) {
    //           mModules[i].setModule(
    //           getTurnAngle(i%2==0?Constants.kDriveWidth/2:-Constants.kDriveWidth/2, i<2?Constants.kDriveLength/2:-Constants.kDriveLength),0);
    //         }
    //     }
    //   return;
    // }
      
    if (controllerAngle == 0 && magnitude == 0) { // Dont set module direction to 0 if not moving
      direction = mLastNonZeroRobotAngle;
    }
    if (controllerAngle != 0)
    mLastNonZeroRobotAngle = controllerAngle;
    
    setVectorSwerveDrive(magnitude, -rotationX, direction);
    mLastGyro = mPeriodicIO.gyro_angle;
  }

  public void jukeMove(boolean jukeRight, boolean jukeLeft) {
    double direction = mLastNonZeroRobotAngle - mPeriodicIO.gyro_angle;
    if (direction < 0) direction += 360;
    int side = (int)(nearestPoleSnap(direction, 45)/90);
    SmartDashboard.putNumber("Side", side);
    if (jukeRight && mRobotCenterX == 0 && mRobotCenterY == 0) {
      mRobotCenterX = Constants.kDriveWidth*(side<2?1:-1);
      mRobotCenterY = Constants.kDriveLength*(side==0||side==3?1:-1);
    }
    else if (jukeLeft && mRobotCenterX == 0 && mRobotCenterY == 0) {
      if (++side > 3) side -= 3;
      mRobotCenterX = Constants.kDriveWidth*(side<2?-1:1);
      mRobotCenterY = Constants.kDriveLength*(side==1||side==3?-1:1);
    }
    else if (!jukeRight && !jukeLeft ) {
      mRobotCenterX = 0;
      mRobotCenterY = 0;
    }
    SmartDashboard.putNumber("Center X", mRobotCenterX);
    SmartDashboard.putNumber("Center Y", mRobotCenterY);
  }
  
  private void setVectorSwerveDrive(double forwardSpeed, double rotationSpeed, double robotAngle) {
    Vector2d[] vectors = new Vector2d[4];
    // vectors[0] = addMovementComponents(forwardSpeed, robotAngle, rotationSpeed, 315);
    // vectors[1] = addMovementComponents(forwardSpeed, robotAngle, rotationSpeed, 225);
    // vectors[2] = addMovementComponents(forwardSpeed, robotAngle, rotationSpeed, 45);
    // vectors[3] = addMovementComponents(forwardSpeed, robotAngle, rotationSpeed, 135);
    for (int i = 0; i < mModules.length; i++) {
      vectors[i] = addMovementComponents(forwardSpeed, robotAngle, rotationSpeed,
      getTurnAngle(((i%2==0?1:-1)*Constants.kDriveWidth/2)-mRobotCenterX, 
                  ((i<2?1:-1)*Constants.kDriveLength/2)-mRobotCenterY) // You need to subtract center
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
  
  // Get angle of a vector 0 is front 90 is right
  private double getVectorAngle(Vector2d v) {
    double angle = Math.toDegrees(Math.atan(v.y / v.x));
    if (v.x == 0) angle = 0;
    if (v.x < 0) angle += 180;
    else if (v.y < 0) angle += 360;
    return angle;
  }

  // Get nearest pole of an angle (N S E W)
  private static double nearestPoleSnap(double angle, double threshold) {
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
    double pole = Math.atan2(poleSin, poleCos);
    if (pole < 0) pole += 2*Math.PI;
    if (Math.toRadians(angle) > Math.PI && poleCos == 1) pole = 2*Math.PI;
    if (Math.abs(pole - Math.toRadians(angle)) <= Math.toRadians(threshold)) {
      double result = Math.toDegrees(Math.atan2(poleSin, poleCos));
      if (result < 0) result += 360;
      return result;
    }
    else 
      return angle;
}

  // Get module rotation angles using x and y position relative to center of robot
  // EX: on a square robot everything is 45 degrees
  private double getTurnAngle(double xPos, double yPos) {
    double theta = Math.toDegrees(Math.atan2(yPos, xPos));
    if (theta > 0) theta -= 360;
    return Math.abs(theta);
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
      mPeriodicIO.drive_velocities[i] = mModules[i].getDriveVelocity();

      mPeriodicIO.rotation_angles[i] = mModules[i].getRotation();
      mPeriodicIO.rotation_deltas[i] = mModules[i].getRotationDelta();
      mPeriodicIO.rotation_velocities[i] = mModules[i].getRotationVelocity();
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
    for (int i = 0; i < mModules.length; i++) {
      mPeriodicIO.module_angles[i] = 0; 
      mPeriodicIO.module_magnitudes[i] = 0;
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
    return new Translation2d(mPeriodicIO.drive_velocities[module]*Math.cos(Math.toRadians(mPeriodicIO.rotation_angles[module])), mPeriodicIO.drive_velocities[module]*Math.sin(Math.toRadians(mPeriodicIO.rotation_angles[module])));
  }

  public double gyroDelta() {
    return mLastGyro - mPeriodicIO.gyro_angle;
  }
}

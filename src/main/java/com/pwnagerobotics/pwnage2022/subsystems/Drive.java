package com.pwnagerobotics.pwnage2022.subsystems;

import com.pwnagerobotics.pwnage2022.Constants;
import com.pwnagerobotics.pwnage2022.lib.SwerveModule;
import com.pwnagerobotics.pwnage2022.subsystems.Drive;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.team254.lib.subsystems.Subsystem;

public class Drive extends Subsystem {
  // Singleton Drive
  public static Drive mInstance;
  public synchronized static Drive getInstance() {
    if (mInstance == null) mInstance = new Drive();
    return mInstance;
  }

  // The encoder values when the motors are straight forwards
  public static class RotationEncoderOffset {
    public static final double kEncoderFrontLeftOffset = 0.601;
    public static final double kEncoderBackLeftOffset = 0.185;
    public static final double kEncoderFrontRightOffset = 0.590;
    public static final double kEncoderBackRightOffset = 0.248;
  }
  
  // Right
  private final MotorController kMotorFrontRightRotation = new PWMMotorController("motor0", 0) {};
  private final MotorController kMotorFrontRightDrive = new PWMMotorController("motor1", 1) {};
  private final MotorController kMotorBackRightRotation = new PWMMotorController("motor2", 2) {};
  private final MotorController kMotorBackRightDrive = new PWMMotorController("motor3", 3) {};
  
  // Left
  private final MotorController kMotorFrontLeftRotation = new PWMMotorController("motor4", 4) {};
  private final MotorController kMotorFrontLeftDrive = new PWMMotorController("motor5", 5) {};
  private final MotorController kMotorBackLeftRotation = new PWMMotorController("motor6", 6) {};
  private final MotorController kMotorBackLeftDrive = new PWMMotorController("motor9", 9) {};
  
  private static final double kControllerDeadband = 0.05;
  private static SwerveModule[] mModules;


  public Drive() {
    mModules[0] = new SwerveModule(Constants.kFrontRightModuleConstants);
    mModules[1] = new SwerveModule(Constants.kFrontLefttModuleConstants);
    mModules[2] = new SwerveModule(Constants.kBackRightModuleConstants);
    mModules[3] = new SwerveModule(Constants.kBackLefttModuleConstants);

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
  
  public synchronized void setRobotCentricSwerveDrive(double throttle, double strafe, double rotation) {
    double angle = Math.toDegrees(Math.atan2(strafe, throttle));
    angle = (angle >= 0) ? angle : angle + 360;
    double speed = Math.sqrt(Math.pow(Math.abs(strafe), 2) + Math.pow(Math.abs(throttle), 2));
    speed = (speed >= 1) ? 1 : speed;
    SmartDashboard.putNumber("Controller angle", angle);
    if (Math.abs(speed) < kControllerDeadband) {
      speed = 0;
      angle = 0;
    }
    updatePID();
    mModules[0].set(angle, speed, rotation);
    mModules[1].set(angle, speed, rotation);
    mModules[2].set(angle, speed, rotation);
    mModules[3].set(angle, speed, rotation);
  }
  
  public synchronized void setFieldCentricSwerveDrive(double throttle, double strafe, double rotation) {
  } 

  private Vector2d scaleVector2d(Vector2d v, double scalar){
    return new Vector2d(v.x * scalar, v.y * scalar);
  }

  private double getVectorAngle(Vector2d v){
    double angle = Math.atan(v.y / v.x) * 180 / 3.14;
    if(v.x < 0){
      angle += 180;
    }else if(v.y < 0){
      angle = 270 - angle;
    }
    return angle;
  }
  
  public void setVectorRobotCentricSwerveDrive(double forwardSpeed, double rotationSpeed, double robotAngle){
    Vector2d FRVector, FLVector, BRVector, BLVector;
    FRVector = addMovementComponents(forwardSpeed, robotAngle, rotationSpeed, 135);
    FLVector = addMovementComponents(forwardSpeed, robotAngle, rotationSpeed, 225);
    BRVector = addMovementComponents(forwardSpeed, robotAngle, rotationSpeed, 45);
    BLVector = addMovementComponents(forwardSpeed, robotAngle, rotationSpeed, 315);
    double maxMagnitude = Math.max(Math.max(Math.max(
      FRVector.magnitude(), 
      FLVector.magnitude()), 
      BLVector.magnitude()),
      BRVector.magnitude());
    if(maxMagnitude > 1){
      //Normalize vectors, preserving proportions while reducing all below 1
      scaleVector2d(FRVector, 1. / maxMagnitude);
      scaleVector2d(FLVector, 1. / maxMagnitude);
      scaleVector2d(BRVector, 1. / maxMagnitude);
      scaleVector2d(BLVector, 1. / maxMagnitude);
    }
    setModule(kMotorFrontLeftDrive, kMotorFrontLeftRotation,
        kEncoderFrontLeftRotation, RotationEncoderOffset.kEncoderFrontLeftOffset,
        getVectorAngle(FLVector), FLVector.magnitude(), kFrontLeftPID);
    setModule(kMotorFrontRightDrive, kMotorFrontRightRotation,
        kEncoderFrontRightRotation, RotationEncoderOffset.kEncoderFrontRightOffset,
        getVectorAngle(FRVector), FRVector.magnitude(), kFrontRightPID);
    setModule(kMotorBackLeftDrive, kMotorBackLeftRotation,
        kEncoderBackLeftRotation, RotationEncoderOffset.kEncoderBackLeftOffset,
        getVectorAngle(BLVector), BLVector.magnitude(), kBackLeftPID);
    setModule(kMotorBackRightDrive, kMotorBackRightRotation,
        kEncoderBackRightRotation, RotationEncoderOffset.kEncoderBackRightOffset,
        getVectorAngle(BRVector), BRVector.magnitude(), kBackRightPID);
  }

  private void setModule(MotorController driveController, MotorController rotationController, AnalogEncoder rotationEncoder,
  double rotationOffset, double rotation, double speed, PIDController pidController)
  {
    // Postion
    double currentPosition = rotationEncoder.getAbsolutePosition() * 360;
    double wantedPosition = rotation + rotationOffset * 360;
    if (wantedPosition > kMaxEncoderValue) wantedPosition -= kMaxEncoderValue;

    // Distance
    double distance = getDistance(currentPosition, wantedPosition);

    // 90 flip
    if (Math.abs(distance) > 90) { // Maybe change to > 90
      wantedPosition -= 180;
      if (wantedPosition < 0) wantedPosition += 1;
      distance = getDistance(currentPosition, wantedPosition);
      speed *= -1;
    }

    // Drive
    driveController.set(speed * mDriveSlowDown);

    // Rotation
    // PID
    if (pidController.atSetpoint()) {
      rotationController.set(0);
    }
    else {
      rotationController.set(pidController.calculate(currentPosition, wantedPosition));
    }
    SmartDashboard.putNumber("PID value", pidController.calculate(currentPosition, wantedPosition));
    // Bang Bang
    // if (Math.abs(rotaionEncoder.getAbsolutePosition() - wantedPosition) > 0.08) {
    //   rotationController.set(1 * mTurnSlowDown * Math.signum(distance));
    // }
    // else {
    //   rotationController.set(0);
    // }
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
      Vector2d forwardVector = new Vector2d(forwardMagnitude * Math.cos(rotation1 * 3.14 / 180), forwardMagnitude * Math.cos(rotation1 * 3.14 / 180));
      Vector2d rotationVector = new Vector2d(rotationalMagnitude * Math.cos(rotation2 * 3.14 / 180), rotationalMagnitude * Math.cos(rotation2 * 3.14 / 180));
      return new Vector2d(forwardVector.x + rotationVector.x, forwardVector.y + rotationVector.y);
  }
  
  private double getDistance(double encoder, double controller) {
    if (controller > encoder) {
      return (encoder - controller) * ((controller - encoder > 180) ? -1 : 1);
    }
    if (encoder > controller) {
      return (encoder - controller) * ((encoder - controller > 180) ? -1 : 1);
    }
    else {
      return 0;
    }
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
    // TODO Auto-generated method stub
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
}

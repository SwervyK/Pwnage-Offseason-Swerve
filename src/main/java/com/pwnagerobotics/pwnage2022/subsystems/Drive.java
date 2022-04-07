package com.pwnagerobotics.pwnage2022.subsystems;

import com.pwnagerobotics.pwnage2022.subsystems.Drive;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;

import com.team254.lib.subsystems.Subsystem;

public class Drive extends Subsystem {
  // Singleton Drive
  public static Drive mInstance;
  public synchronized static Drive getInstance() {
    if (mInstance == null) mInstance = new Drive();
    return mInstance;
  }

  // Zeros
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
  
  // Rotation Encoders
  private static final AnalogEncoder kEncoderFrontLeftRotation = new AnalogEncoder(0);
  private static final AnalogEncoder kEncoderBackLeftRotation = new AnalogEncoder(1);
  private static final AnalogEncoder kEncoderFrontRightRotation = new AnalogEncoder(2);
  private static final AnalogEncoder kEncoderBackRightRotation = new AnalogEncoder(3);

  //PID
  private static final PIDController kFrontLeftPID = new PIDController(0.7, 0.015, 0);
  private static final PIDController kBackLeftPID = new PIDController(0.7, 0.015, 0);
  private static final PIDController kFrontRightPID = new PIDController(0.7, 0.015, 0);
  private static final PIDController kBackRightPID = new PIDController(0.7, 0.015, 0);
  
  private static final double kMaxEncoderValue = 1.0; // 0.974 OLD
  private double mDriveSlowDown = 0.5;
  private double mTurnSlowDown = 0.5;
  
  public Drive() {
      kFrontLeftPID.enableContinuousInput(0, kMaxEncoderValue*360);
      kBackLeftPID.enableContinuousInput(0, kMaxEncoderValue*360);
      kFrontRightPID.enableContinuousInput(0, kMaxEncoderValue*360);
      kBackRightPID.enableContinuousInput(0, kMaxEncoderValue*360);
  }

  public synchronized void setRobotCentricSwerveDrive(double throttle, double strafe, double rotation) {
    double angle = Math.toDegrees(Math.atan2(strafe, throttle));
    angle = (angle >= 0) ? angle : angle + 360;
    double speed = Math.sqrt(Math.pow(Math.abs(strafe), 2) + Math.pow(Math.abs(throttle), 2));
    speed = (speed >= 1) ? 1 : speed;
    SmartDashboard.putNumber("Controller angle", angle);
    // // If not moving have wheels rotate to make harder to push
    // if (throttle == 0 && strafe == 0) {
    //   angle = 0.25/2;
    //   setModule(kMotorFrontLeftDrive, kMotorFrontLeftRotation, kEncoderFrontLeftRotation, RotationEncoderOffset.kEncoderFrontLeftOffset, angle, 0, kFrontLeftPID);
    //   angle = 0.25/2 + 0.75;
    //   setModule(kMotorBackLeftDrive, kMotorBackLeftRotation, kEncoderBackLeftRotation, RotationEncoderOffset.kEncoderBackLeftOffset, angle, 0, kBackLeftPID);
    //   angle = 0.25/2 + 0.25;
    //   setModule(kMotorFrontRightDrive, kMotorFrontRightRotation, kEncoderFrontRightRotation, RotationEncoderOffset.kEncoderFrontRightOffset, angle, 0, kFrontRightPID);
    //   angle = 0.25/2 + 0.50;
    //   setModule(kMotorBackRightDrive, kMotorBackRightRotation, kEncoderBackRightRotation, RotationEncoderOffset.kEncoderBackRightOffset, angle, 0, kBackRightPID);
    // }
    // // Rotation
    // else if (rotation > 0) {
    //   angle = 0.25/2;
    //   setModule(kMotorFrontLeftDrive, kMotorFrontLeftRotation, kEncoderFrontLeftRotation, RotationEncoderOffset.kEncoderFrontLeftOffset, angle, -rotation, kFrontLeftPID);
    //   angle = 0.25/2 + 0.75;
    //   setModule(kMotorBackLeftDrive, kMotorBackLeftRotation, kEncoderBackLeftRotation, RotationEncoderOffset.kEncoderBackLeftOffset, angle, rotation, kBackLeftPID);
    //   angle = 0.25/2 + 0.25;
    //   setModule(kMotorFrontRightDrive, kMotorFrontRightRotation, kEncoderFrontRightRotation, RotationEncoderOffset.kEncoderFrontRightOffset, angle, -rotation, kFrontRightPID);
    //   angle = 0.25/2 + 0.50;
    //   setModule(kMotorBackRightDrive, kMotorBackRightRotation, kEncoderBackRightRotation, RotationEncoderOffset.kEncoderBackRightOffset, angle, -rotation, kBackRightPID);
    // }
    // // Throttle/Strafe
    // else {
      setModule(kMotorFrontLeftDrive, kMotorFrontLeftRotation, kEncoderFrontLeftRotation, RotationEncoderOffset.kEncoderFrontLeftOffset, angle, -speed, kFrontLeftPID);
      setModule(kMotorBackLeftDrive, kMotorBackLeftRotation, kEncoderBackLeftRotation, RotationEncoderOffset.kEncoderBackLeftOffset, angle, speed, kBackLeftPID);
      setModule(kMotorFrontRightDrive, kMotorFrontRightRotation, kEncoderFrontRightRotation, RotationEncoderOffset.kEncoderFrontRightOffset, angle, -speed, kFrontRightPID);
      setModule(kMotorBackRightDrive, kMotorBackRightRotation, kEncoderBackRightRotation, RotationEncoderOffset.kEncoderBackRightOffset, angle, -speed, kBackRightPID);
    // }
  }

  public synchronized void setFieldCentricSwerveDrive(double throttle, double strafe, double rotation) {
  } 

  private Vector2d addVector2d(Vector2d v1, Vector2d v2){
    return new Vector2d(v1.x + v2.x, v1.y + v2.y);
  }

  private Vector2d scaleVector2d(Vector2d v, double scalar){
    return new Vector2d(v.x * scalar, v.y * scalar);
  }

  private double getVectorAngle(Vector2d v){
    double angle = Math.atan(v.y / v.x) * 180 / 3.14;
    if(v.x < 0){
      angle += 180;
    }else if(v.y < 0){
      angle = 180 - angle;
    }
    return angle;
  }
  
  public void setVectorBasedSwerveDrive(double forwardSpeed, double rotationSpeed, double robotAngle){
    Vector2d FRVector, FLVector, BRVector, BLVector;
    FRVector = addMovementComponents(forwardSpeed, robotAngle, rotationSpeed, 135);
    FLVector = addMovementComponents(forwardSpeed, robotAngle, rotationSpeed, 225);
    BRVector = addMovementComponents(forwardSpeed, robotAngle, rotationSpeed, 45);
    BLVector = addMovementComponents(forwardSpeed, robotAngle, rotationSpeed, 315);
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

  private Vector2d addMovementComponents(double forwardMagnitude, double rotation1, double rotationalMagnitude, double rotation2){
      Vector2d forwardVector = new Vector2d(forwardMagnitude * Math.cos(rotation2 * 3.14 / 180), forwardMagnitude * Math.cos(rotation2 * 3.14 / 180));
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
    SmartDashboard.putNumber("Front Left", kEncoderFrontLeftRotation.getAbsolutePosition());
    SmartDashboard.putNumber("Back Left", kEncoderBackLeftRotation.getAbsolutePosition());
    SmartDashboard.putNumber("Front Right", kEncoderFrontRightRotation.getAbsolutePosition());
    SmartDashboard.putNumber("Back Right", kEncoderBackRightRotation.getAbsolutePosition());
  }
  
  @Override
  public boolean checkSystem() {
    // TODO Auto-generated method stub
    return false;
  }
}

package com.pwnagerobotics.pwnage2022.subsystems;

import com.pwnagerobotics.pwnage2022.subsystems.Drive;

import edu.wpi.first.wpilibj.AnalogEncoder;
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
  
  private static final double kMaxEncoderValue = 0.974;
  private double mRotationError = 0.08;
  private double mDriveSlowDown = 1;
  private double mTurnSlowDown = 0.8;
  
  public synchronized void setSwerveDrive(double throttle, double strafe, double rotation) {
    double angle = Math.toDegrees(Math.atan2(strafe, throttle));
    angle = (angle >= 0) ? angle : angle + 360;
    angle /= 360;
    double speed = Math.sqrt(Math.pow(Math.abs(strafe), 2) + Math.pow(Math.abs(throttle), 2));
    speed = (speed >= 1) ? 1 : speed;
    
    if (rotation > 0) {
      angle = 0.25/2;
      setModule(kMotorFrontLeftDrive, kMotorFrontLeftRotation, kEncoderFrontLeftRotation, RotationEncoderOffset.kEncoderFrontLeftOffset, angle, -rotation, 0);
      angle = 0.25/2 + 0.75;
      setModule(kMotorBackLeftDrive, kMotorBackLeftRotation, kEncoderBackLeftRotation, RotationEncoderOffset.kEncoderBackLeftOffset, angle, rotation, 1);
      angle = 0.25/2 + 0.25;
      setModule(kMotorFrontRightDrive, kMotorFrontRightRotation, kEncoderFrontRightRotation, RotationEncoderOffset.kEncoderFrontRightOffset, angle, -rotation, 3);
      angle = 0.25/2 + 0.50;
      setModule(kMotorBackRightDrive, kMotorBackRightRotation, kEncoderBackRightRotation, RotationEncoderOffset.kEncoderBackRightOffset, angle, -rotation, 4);
    }
    else {
      setModule(kMotorFrontLeftDrive, kMotorFrontLeftRotation, kEncoderFrontLeftRotation, RotationEncoderOffset.kEncoderFrontLeftOffset, angle, -speed, 0);
      setModule(kMotorBackLeftDrive, kMotorBackLeftRotation, kEncoderBackLeftRotation, RotationEncoderOffset.kEncoderBackLeftOffset, angle, speed, 1);
      setModule(kMotorFrontRightDrive, kMotorFrontRightRotation, kEncoderFrontRightRotation, RotationEncoderOffset.kEncoderFrontRightOffset, angle, -speed, 3);
      setModule(kMotorBackRightDrive, kMotorBackRightRotation, kEncoderBackRightRotation, RotationEncoderOffset.kEncoderBackRightOffset, angle, -speed, 4);
    }
  }
  
  private void setModule(MotorController driveController, MotorController rotationConstrooler, AnalogEncoder rotaionEncoder,
  double rotationOffset, double rotation, double speed, int id) 
  {
    double wantedPosition = getMotorRotation(rotation, rotationOffset, rotaionEncoder.getAbsolutePosition());
    double distance = getDistance(rotaionEncoder.getAbsolutePosition(), wantedPosition, id);
    speed *= mDriveSlowDown;
    driveController.set(speed);
    if (Math.abs(rotaionEncoder.getAbsolutePosition() - wantedPosition) > mRotationError) {
      rotationConstrooler.set(mTurnSlowDown * Math.signum(distance));
    }
    else {
      rotationConstrooler.set(0);
    }
  }
  
  private double getDistance(double encoder, double controller, int id) {
    if (controller > encoder) {
      return (encoder - controller) * ((controller - encoder > 0.5) ? -1 : 1);
    }
    if (encoder > controller) {
      return (encoder - controller) * ((encoder - controller > 0.5) ? -1 : 1);
    }
    else {
      return 0;
    }
  }
  
  private double getMotorRotation(double value, double zero, double encoder) {
    value += zero;
    if (value > kMaxEncoderValue) {
      value -= kMaxEncoderValue;
    }
    return value;
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

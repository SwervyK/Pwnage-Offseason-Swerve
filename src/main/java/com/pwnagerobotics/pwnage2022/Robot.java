// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.pwnagerobotics.pwnage2022;

import com.pwnagerobotics.pwnage2022.humans.driver.XboxDriver;
import com.team254.lib.util.LatchedBoolean;

import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  public static class RotationEncoderOffset {
    public static final double mEncoderFrontLeftOffset = 0.601;
    public static final double mEncoderBackLeftOffset = 0.185;
    public static final double mEncoderFrontRightOffset = 0.590;
    public static final double mEncoderBackRightOffset = 0.248;
  }

  // Right
  public MotorController mMotorFrontRightRotation = new PWMMotorController("motor0", 0) {};
  public MotorController mMotorFrontRightDrive = new PWMMotorController("motor1", 1) {};
  public MotorController mMotorBackRightRotation = new PWMMotorController("motor2", 2) {};
  public MotorController mMotorBackRightDrive = new PWMMotorController("motor3", 3) {};

  // Left
  public MotorController mMotorFrontLeftRotation = new PWMMotorController("motor4", 4) {};
  public MotorController mMotorFrontLeftDrive = new PWMMotorController("motor5", 5) {};
  public MotorController mMotorBackLeftRotation = new PWMMotorController("motor6", 6) {};
  public MotorController mMotorBackLeftDrive = new PWMMotorController("motor9", 9) {};

  // Rotation Encoders
  private AnalogEncoder mEncoderFrontLeftRotation = new AnalogEncoder(0);
  private AnalogEncoder mEncoderBackLeftRotation = new AnalogEncoder(1);
  private AnalogEncoder mEncoderFrontRightRotation = new AnalogEncoder(2);
  private AnalogEncoder mEncoderBackRightRotation = new AnalogEncoder(3);

  // Pnumatics
  public Solenoid driveShift = new Solenoid(PneumaticsModuleType.REVPH, 0);
  private LatchedBoolean mToggleShift = new LatchedBoolean();
  private boolean mSetShift = false;

  // Drive
  public double mSpeed = 0.0;
  public double mRotation = 0.0;
  public double deadband = 0.1;
  public double error = 0.08;
  public double slowDown = 1;
  public double turnSpeed = 0.8;
  public static final double kMaxValue = 0.974;


  public Robot(){
    super(0.04);
  }

  // Humans
  private XboxDriver mDriver = new XboxDriver();

  @Override
  public void teleopInit() {

  }

  @Override
  public void teleopPeriodic() {

    if (Math.abs(mDriver.getPositionX()) > deadband || Math.abs(mDriver.getPositionY()) > deadband) {
      double angle = Math.toDegrees(Math.atan2(mDriver.getPositionX(), mDriver.getPositionY()));
      double speed = Math.sqrt(Math.pow(Math.abs(mDriver.getPositionX()), 2) + Math.pow(Math.abs(mDriver.getPositionY()), 2));
      angle = (angle > 0) ? angle : angle + 360;
      speed = (speed > 1) ? 1 : speed;
      angle /= 360;
      setDrive(speed, angle);
    }
    else if (mDriver.getDPad() == 0) {
      setDrive(0, 0);
    }
    else {
      setDrive(0, -1);
    }

    if (Math.abs(mDriver.getRotationX()) > deadband) {
      rotate(-mDriver.getRotationX());
    }
    else {
      runDrive();
    }
  }

  public void runDrive() {
    mSpeed *= slowDown;
    mMotorFrontRightDrive.set(-mSpeed);
    mMotorBackRightDrive.set(-mSpeed);
    mMotorFrontLeftDrive.set(-mSpeed);
    mMotorBackLeftDrive.set(mSpeed);

    if (Math.abs(mEncoderFrontRightRotation.getAbsolutePosition() - getMotorRotation(mRotation, RotationEncoderOffset.mEncoderFrontRightOffset, mEncoderFrontRightRotation.getAbsolutePosition())) > error) {
      mMotorFrontRightRotation.set(turnSpeed * getSign(mEncoderFrontRightRotation.getAbsolutePosition(), getMotorRotation(mRotation, RotationEncoderOffset.mEncoderFrontRightOffset, mEncoderFrontRightRotation.getAbsolutePosition()), 0));
    }
    else {
      mMotorFrontRightRotation.set(0);
    }
    if (Math.abs(mEncoderBackRightRotation.getAbsolutePosition() - getMotorRotation(mRotation, RotationEncoderOffset.mEncoderBackRightOffset, mEncoderBackRightRotation.getAbsolutePosition())) > error) {
      mMotorBackRightRotation.set(turnSpeed * getSign(mEncoderBackRightRotation.getAbsolutePosition(), getMotorRotation(mRotation, RotationEncoderOffset.mEncoderBackRightOffset, mEncoderBackRightRotation.getAbsolutePosition()), 1));
    }
    else {
      mMotorBackRightRotation.set(0);
    }
    if (Math.abs(mEncoderFrontLeftRotation.getAbsolutePosition() - getMotorRotation(mRotation, RotationEncoderOffset.mEncoderFrontLeftOffset, mEncoderFrontLeftRotation.getAbsolutePosition())) > error) {
      mMotorFrontLeftRotation.set(turnSpeed * getSign(mEncoderBackRightRotation.getAbsolutePosition(), getMotorRotation(mRotation, RotationEncoderOffset.mEncoderBackRightOffset, mEncoderFrontLeftRotation.getAbsolutePosition()), 2));
    }
    else {
      mMotorFrontLeftRotation.set(0);
    }
    if (Math.abs(mEncoderBackLeftRotation.getAbsolutePosition() - getMotorRotation(mRotation, RotationEncoderOffset.mEncoderBackLeftOffset, mEncoderBackLeftRotation.getAbsolutePosition())) > error) {
      mMotorBackLeftRotation.set(turnSpeed * getSign(mEncoderBackRightRotation.getAbsolutePosition(), getMotorRotation(mRotation, RotationEncoderOffset.mEncoderBackRightOffset, mEncoderBackLeftRotation.getAbsolutePosition()), 3));
    }
    else {
      mMotorBackLeftRotation.set(0);
    }
  }

  public double getDistance(double encoder, double controller, int id) {
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

  public double getSign(double encoder, double controller, int id) {
    return Math.signum(getDistance(encoder, controller, id));
  }

  public void rotate(double speed) {
    speed *= -slowDown;
    mMotorFrontRightDrive.set(-speed);
    mMotorBackRightDrive.set(-speed);
    mMotorFrontLeftDrive.set(-speed);
    mMotorBackLeftDrive.set(speed);
    mRotation = 0.25/2 + 0.25;
    if (Math.abs(mEncoderFrontRightRotation.getAbsolutePosition() - getMotorRotation(mRotation, RotationEncoderOffset.mEncoderFrontRightOffset, mEncoderFrontRightRotation.getAbsolutePosition())) > error) {
      mMotorFrontRightRotation.set(turnSpeed * getSign(mEncoderFrontRightRotation.getAbsolutePosition(), getMotorRotation(mRotation, RotationEncoderOffset.mEncoderFrontRightOffset, mEncoderFrontRightRotation.getAbsolutePosition()), 0));
    }
    else {
      mMotorBackRightRotation.set(0);
    }
    mRotation = 0.25/2 + 0.50;
    if (Math.abs(mEncoderBackRightRotation.getAbsolutePosition() - getMotorRotation(mRotation, RotationEncoderOffset.mEncoderBackRightOffset, mEncoderBackRightRotation.getAbsolutePosition())) > error) {
      mMotorBackRightRotation.set(turnSpeed * getSign(mEncoderBackRightRotation.getAbsolutePosition(), getMotorRotation(mRotation, RotationEncoderOffset.mEncoderBackRightOffset, mEncoderBackRightRotation.getAbsolutePosition()), 1));
    }
    else {
      mMotorBackRightRotation.set(0);
    }
    mRotation = 0.25/2;
    if (Math.abs(mEncoderFrontLeftRotation.getAbsolutePosition() - getMotorRotation(mRotation, RotationEncoderOffset.mEncoderFrontLeftOffset, mEncoderFrontLeftRotation.getAbsolutePosition())) > error) {
      mMotorFrontLeftRotation.set(turnSpeed * getSign(mEncoderBackRightRotation.getAbsolutePosition(), getMotorRotation(mRotation, RotationEncoderOffset.mEncoderBackRightOffset, mEncoderFrontLeftRotation.getAbsolutePosition()), 1));
    }
    else {
      mMotorFrontLeftRotation.set(0);
    }
    mRotation = 0.25/2 + 0.75;
    if (Math.abs(mEncoderBackLeftRotation.getAbsolutePosition() - getMotorRotation(mRotation, RotationEncoderOffset.mEncoderBackLeftOffset, mEncoderBackLeftRotation.getAbsolutePosition())) > error) {
      mMotorBackLeftRotation.set(turnSpeed * getSign(mEncoderBackRightRotation.getAbsolutePosition(), getMotorRotation(mRotation, RotationEncoderOffset.mEncoderBackRightOffset, mEncoderBackLeftRotation.getAbsolutePosition()), 1));
    }
    else {
      mMotorBackLeftRotation.set(0);
    }
  }

  private void setDrive(double speed, double rotation) {
    mSpeed = speed;
    if (rotation != -1)
      mRotation = rotation;
  }

  private double getMotorRotation(double value, double zero, double encoder) {
    value += zero;
    if (value > kMaxValue) {
      value -= kMaxValue;
    }
    return value;
  }

  @Override
  public void robotInit() {

  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Front Left", mEncoderFrontLeftRotation.getAbsolutePosition());
    SmartDashboard.putNumber("Back Left", mEncoderBackLeftRotation.getAbsolutePosition());
    SmartDashboard.putNumber("Front Right", mEncoderFrontRightRotation.getAbsolutePosition());
    SmartDashboard.putNumber("Back Right", mEncoderBackRightRotation.getAbsolutePosition());
    SmartDashboard.putBoolean("Shifter", mSetShift);
  }

  @Override
  public void disabledInit() {

  }

  @Override
  public void disabledPeriodic() {
    
  }

  @Override
  public void autonomousInit() { }

  @Override
  public void autonomousPeriodic() { }


  @Override
  public void testInit() { }

  @Override
  public void testPeriodic() { }
}

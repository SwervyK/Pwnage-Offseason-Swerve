// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.pwnagerobotics.pwnage2022;

import org.opencv.core.Mat;

import com.pwnagerobotics.pwnage2022.auto.Action;
import com.pwnagerobotics.pwnage2022.auto.Action.RobotState;
import com.pwnagerobotics.pwnage2022.auto.Autos;
import com.pwnagerobotics.pwnage2022.auto.Playback;
import com.pwnagerobotics.pwnage2022.auto.Recorder;
import com.pwnagerobotics.pwnage2022.humans.driver.XboxDriver;
import com.pwnagerobotics.pwnage2022.subsystems.Drive;
import com.pwnagerobotics.pwnage2022.subsystems.Drive.DriveMode;
import com.pwnagerobotics.pwnage2022.subsystems.Drive.RotationMode;
import com.pwnagerobotics.pwnage2022.subsystems.RobotStateEstimator;
import com.team254.lib.drivers.LazySparkMax;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.geometry.Twist2d;
import com.team254.lib.subsystems.SubsystemManager;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;


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
  
  // Subsystems
  private final SubsystemManager mSubsystemManager = SubsystemManager.getInstance();
  private final Drive mDrive = Drive.getInstance();
  private final RobotStateEstimator mRobotStateEstimator = RobotStateEstimator.getInstance();
  
  // Auto
  //private final Recorder mRecorder = new Recorder();
  //private final Playback mPlayback = new Playback();
  //private double startPlayback = 0;
  //private double startRecording = 0;
  
  private enum AutoType {
    RECORDER,
    PLAYBACK
  }
  private enum TuningMode {
    ROBOT,
    MODULE
  }

  // CONFIG
  private boolean isRecordingAuto = false;
  private TuningMode mTuningMode = TuningMode.ROBOT;
  private AutoType mAutoType = AutoType.RECORDER;
  private Action[] mCurrentAuto = Autos.square();
  // CONFIG

  public Robot(){
    super(0.04);
  }
  
  // Humans
  private XboxDriver mDriver = new XboxDriver();
  
  @Override
  public void robotInit() {
    mSubsystemManager.setSubsystems(
    mDrive,
    mRobotStateEstimator
    );
    
    mDrive.zeroSensors();
  }
  
  @Override
  public void robotPeriodic() {
    mSubsystemManager.outputToSmartDashboard();
  }
  
  @Override
  public void disabledInit() {
    var timestamp = Timer.getFPGATimestamp();
    mSubsystemManager.executeEnabledLoopStops(timestamp);
    mRobotStateEstimator.onEnabledLoopStart(timestamp);
  }
  
  @Override
  public void disabledPeriodic() {
    var timestamp = Timer.getFPGATimestamp();
    mSubsystemManager.executeDisabledLoops(timestamp);
    mRobotStateEstimator.onEnabledLoop(timestamp);
  }
  
  @Override
  public void autonomousInit() { 
    // var timestamp = Timer.getFPGATimestamp();
    // mSubsystemManager.executeEnabledLoopStarts(timestamp);
    
    // if (mAutoType == AutoType.PLAYBACK) {
    //   mPlayback.setActions(mCurrentAuto, true);
    // }
    // startPlayback = timestamp;
  }
  
  @Override
  public void autonomousPeriodic() {
    // var timestamp = Timer.getFPGATimestamp();
    
    // Action action = new Action(new RobotState(0, 0, 0), false);
    // if (mAutoType == AutoType.RECORDER) {
    //   action = mRecorder.getSwerveStateAtTimestamp(timestamp - startPlayback);
    // }
    // else if (mAutoType == AutoType.PLAYBACK) {
    //   action = mPlayback.getCurrentAction();
    // }
    
    // mDrive.setRotationMode(action.isFieldCentricRotation() ? RotationMode.FIELD : RotationMode.ROBOT);
    // mDrive.setSwerveDrive(action.getDrive()[0], action.getDrive()[1], action.getRotation()[0], action.getRotation()[1]);
    
    // mSubsystemManager.executeEnabledLoops(Timer.getFPGATimestamp());
  }
  
  @Override
  public void teleopInit() {
    mSubsystemManager.executeEnabledLoopStops(Timer.getFPGATimestamp());
    mSubsystemManager.executeEnabledLoopStarts(Timer.getFPGATimestamp());
    
    // if (isRecordingAuto) {
    //   mRecorder.newAuto("spin");
    //   startRecording = Timer.getFPGATimestamp();
    // }
  }
  
  @Override
  public void teleopPeriodic() {
    var timestamp = Timer.getFPGATimestamp();
    
    double rotationX = mDriver.getRotationX();
    double rotationY = mDriver.getRotationY();
    double throttle = mDriver.getPositionY();
    double strafe = mDriver.getPositionX();
    boolean wantFieldCentricDrive = !mDriver.wantFieldCentricDrive();
    boolean wantFieldCentricRotation = mDriver.wantFieldCentricRotation();
    boolean jukeRight = mDriver.wantJukeRight();
    boolean jukeLeft = mDriver.wantJukeLeft();
    boolean wantGyroReset = mDriver.getDPad() == 180;
    
    mDrive.setDriveMode(wantFieldCentricDrive ? DriveMode.FIELD : DriveMode.ROBOT);
    mDrive.jukeMove(jukeRight, jukeLeft);
    mDrive.setRotationMode(wantFieldCentricRotation ? RotationMode.FIELD : RotationMode.ROBOT);
    mDrive.setKinematicsDrive(throttle, strafe, rotationX, rotationY);
    
    if (wantGyroReset) {
      mDrive.zeroSensors();
    }
    
    // if (mRecorder.isRecording()) {
    //   boolean stopRecording = mDriver.getDPad() == 0;
    //   mRecorder.recordInputs(throttle, strafe, rotationX, rotationY, wantFieldCentricRotation, timestamp - startRecording);
    //   if (stopRecording) {
    //     mRecorder.stopRecording();
    //   }
    // }
    
    mSubsystemManager.executeEnabledLoops(timestamp);
  }
  
  @Override
  public void testInit() {
    double x = 1;
    double y = 1;
    Rotation2d rotation = new Rotation2d(Math.PI, false); // Radians

    {
      Pose2d robotVelocity = new Pose2d(x, y, rotation);
      Translation2d[] modules = Kinematics.inverseKinematics(robotVelocity);
      System.out.println("Kinematics.inverseKinematics");
      System.out.println("Angle: " + modules[0].direction().getDegrees()); // front right
      System.out.println("Mag: " + modules[0].norm());
    }
    {
      Translation2d[] moduleStates = new Translation2d[4];
      for (int i = 0; i < moduleStates.length; i++) moduleStates[i] = new Translation2d(x, y);
      Pose2d robot  = Kinematics.forwardKinematics(moduleStates);
      System.out.println("Kinematics.forwardKinematics Pose");
      System.out.println("X: " + robot.getTranslation().x());
      System.out.println("Y: " + robot.getTranslation().y());
      System.out.println("rot" + robot.getRotation().getDegrees());
    }
    {
      double[] wheel_speeds = new double[4];
      Rotation2d[] wheel_azimuths = new Rotation2d[4];
      for (int i = 0; i < wheel_speeds.length; i++) {
        wheel_speeds[i] = Math.hypot(x, y);
        wheel_azimuths[i] = new Rotation2d(x, y, false);
      } 
      Twist2d robot  = Kinematics.forwardKinematics(wheel_speeds, wheel_azimuths);
      System.out.println("Kinematics.forwardKinematics Twist");
      System.out.println("X: " + robot.dx);
      System.out.println("Y: " + robot.dy);
      System.out.println("rot" + robot.dtheta);
    }
  }

  @Override
  public void testPeriodic() {
    // var timestamp = Timer.getFPGATimestamp();
    // // Robot Rotation PID tuning
    // if (mTuningMode == TuningMode.ROBOT) {
    //   double throttle = mDriver.getPositionY();
    //   double strafe = mDriver.getPositionX();
    //   int dPad = mDriver.getDPad();
      
    //   mDrive.setRotationMode(RotationMode.FIELD);
    //   mDrive.setSwerveDrive(throttle, strafe, Math.toDegrees(Math.sqrt(2)*Math.cos(Math.toRadians(dPad))), Math.toDegrees(Math.sqrt(2)*Math.sin(Math.toRadians(dPad))));
    //   System.out.println("X: " + Math.sqrt(2)*Math.toDegrees(Math.cos(Math.toRadians(dPad))) + " | Y: " + Math.sqrt(2)*Math.toDegrees(Math.sin(Math.toRadians(dPad)))); //TODO I dont think the X and Y are correct
    // }
    // else if (mTuningMode == TuningMode.MODULE) {
   //   // Module Rotation PID tuning
      // double direction = Math.toDegrees(Math.atan2(mDriver.getPositionX(), mDriver.getPositionY()));
      // if (direction < 0) direction += 360;
      // double magnitude = Math.hypot(Math.abs(mDriver.getPositionX()), Math.abs(mDriver.getPositionY())); // Get wanted speed of robot

      // mDrive.setModule(0, direction, magnitude);
      // mDrive.setModule(1, direction, magnitude);
      // mDrive.setModule(2, direction, magnitude);
      // mDrive.setModule(3, direction, magnitude);
      // mSubsystemManager.executeEnabledLoops(timestamp);
    // }
  }
}

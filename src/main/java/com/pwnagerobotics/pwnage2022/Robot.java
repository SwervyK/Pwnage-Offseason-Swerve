// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.pwnagerobotics.pwnage2022;

import com.pwnagerobotics.pwnage2022.auto.Action;
import com.pwnagerobotics.pwnage2022.auto.Action.RobotState;
import com.pwnagerobotics.pwnage2022.auto.Autos;
import com.pwnagerobotics.pwnage2022.auto.Playback;
import com.pwnagerobotics.pwnage2022.auto.Recorder;
import com.pwnagerobotics.pwnage2022.humans.driver.XboxDriver;
import com.pwnagerobotics.pwnage2022.lib.SwerveModule;
import com.pwnagerobotics.pwnage2022.subsystems.Drive;
import com.pwnagerobotics.pwnage2022.subsystems.Drive.DriveMode;
import com.pwnagerobotics.pwnage2022.subsystems.Drive.RotationMode;
import com.team254.lib.subsystems.SubsystemManager;

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
  
  // Subsystemss
  private final SubsystemManager mSubsystemManager = SubsystemManager.getInstance();
  private final Drive mDrive = Drive.getInstance();
  
  // Auto
  private final Recorder mRecorder = new Recorder();
  private final Playback mPlayback = new Playback();
  private double startPlayback = 0;
  private double startRecording = 0;
  
  private enum AutoType {
    RECORDER,
    PLAYBACK
  }
  private AutoType mAutoType = AutoType.RECORDER;
  
  public Robot(){
    super(0.04);
  }
  
  // Humans
  private XboxDriver mDriver = new XboxDriver();
  
  @Override
  public void robotInit() {
    mSubsystemManager.setSubsystems(
    mDrive
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
  }
  
  @Override
  public void disabledPeriodic() {
    var timestamp = Timer.getFPGATimestamp();
    mSubsystemManager.executeDisabledLoops(timestamp);
  }
  
  @Override
  public void autonomousInit() { 
    var timestamp = Timer.getFPGATimestamp();
    mSubsystemManager.executeEnabledLoopStarts(timestamp);
    
    if (mAutoType == AutoType.PLAYBACK) {
      mPlayback.startAction(Autos.square(), true);
    }
    startPlayback = timestamp;
  }
  
  @Override
  public void autonomousPeriodic() {
    var timestamp = Timer.getFPGATimestamp();
    
    Action action = new Action(new RobotState(0, 0, 0), false);
    if (mAutoType == AutoType.RECORDER) {
      action = mRecorder.getSwerveStateAtTimestamp(timestamp - startPlayback);
    }
    else if (mAutoType == AutoType.PLAYBACK) {
      action = mPlayback.getCurrentAction();
    }
    
    mDrive.setRotationMode(action.getFieldCentricRotation() ? RotationMode.FEILD : RotationMode.ROBOT);
    mDrive.setSwerveDrive(action.getDrive()[0], action.getDrive()[1], action.getRotation()[0], action.getRotation()[1]);
    
    mSubsystemManager.executeEnabledLoops(Timer.getFPGATimestamp());
  }
  
  @Override
  public void teleopInit() {
    mSubsystemManager.executeEnabledLoopStops(Timer.getFPGATimestamp());
    mSubsystemManager.executeEnabledLoopStarts(Timer.getFPGATimestamp());
    
    //mRecorder.newAuto("spin");
    //startRecording = Timer.getFPGATimestamp();
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
    boolean wantGyroReset = mDriver.getDPad() == 180;
    
    mDrive.setDriveMode(wantFieldCentricDrive ? DriveMode.FEILD : DriveMode.ROBOT);
    mDrive.setRotationMode(wantFieldCentricRotation ? RotationMode.FEILD : RotationMode.ROBOT);
    mDrive.setSwerveDrive(throttle, strafe, rotationX, rotationY);
    
    if (wantGyroReset) {
      mDrive.zeroSensors();
    }
    
    if (mRecorder.isRecording()) {
      boolean stopRecording = mDriver.getDPad() == 0;
      mRecorder.recordInputs(throttle, strafe, rotationX, rotationY, wantFieldCentricRotation, timestamp - startRecording);
      if (stopRecording) {
        mRecorder.stopRecording();
      }
    }
    
    mSubsystemManager.executeEnabledLoops(timestamp);
  }
  
  @Override
  public void testInit() { }
  
  @Override
  public void testPeriodic() {
    // Robot Rotation PID tuning
    // double throttle = mDriver.getPositionY();
    // double strafe = mDriver.getPositionX();
    // int dPad = mDriver.getDPad();
    
    // mDrive.setRotationMode(RotationMode.FEILD);
    // mDrive.setSwerveDrive(throttle, strafe, Math.cos(Math.toRadians(dPad)), Math.sin(Math.toRadians(dPad)));
    // System.out.println("X: " + Math.cos(Math.toRadians(dPad)) + " | Y: " + Math.sin(Math.toRadians(dPad))); //TODO I dont think the X and Y are correct
    
    // Drive Roation PID tuning
    // int dPad = mDriver.getDPad(); // Cardinal Directions
    // mDrive.setModule(0, dPad, 0);
    // mDrive.setModule(1, dPad, 0);
    // mDrive.setModule(2, dPad, 0);
    // mDrive.setModule(3, dPad, 0);
    
  }
}

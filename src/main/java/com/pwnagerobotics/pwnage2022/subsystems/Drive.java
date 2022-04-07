package com.pwnagerobotics.pwnage2022.subsystems;

import com.pwnagerobotics.pwnage2022.Constants;
import com.pwnagerobotics.pwnage2022.lib.SwerveModule;
import com.pwnagerobotics.pwnage2022.subsystems.Drive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.team254.lib.subsystems.Subsystem;

public class Drive extends Subsystem {
  // Singleton Drive
  public static Drive mInstance;
  public synchronized static Drive getInstance() {
    if (mInstance == null) mInstance = new Drive();
    return mInstance;
  }
  
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

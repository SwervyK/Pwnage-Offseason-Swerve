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

    // Drive
    private static final double kMaxValue = 0.974;
    private double mSpeed = 0.0;
    private double mRotation = 0.0;
    private double mDeadband = 0.1;
    private double mError = 0.08;
    private double mSlowDown = 1;
    private double mTurnSpeed = 0.8;

    public synchronized void setSwerveDrive(double throttle, double strafe, double rotation) {
        if (Math.abs(strafe) > mDeadband || Math.abs(throttle) > mDeadband) {
            double angle = Math.toDegrees(Math.atan2(strafe, throttle));
            double speed = Math.sqrt(Math.pow(Math.abs(strafe), 2) + Math.pow(Math.abs(throttle), 2));
            angle = (angle > 0) ? angle : angle + 360;
            speed = (speed > 1) ? 1 : speed;
            angle /= 360;
            setDrive(speed, angle);
          }
          else {
            setDrive(0, -1);
          }
      
          if (Math.abs(rotation) > mDeadband) {
            rotate(-rotation);
          }
          else {
            runDrive();
          }
    }

    public synchronized void zeroDrive() {
        setDrive(0, 0);
    }

    public void runDrive() {
        mSpeed *= mSlowDown;
        kMotorFrontRightDrive.set(-mSpeed);
        kMotorBackRightDrive.set(-mSpeed);
        kMotorFrontLeftDrive.set(-mSpeed);
        kMotorBackLeftDrive.set(mSpeed);
    
        if (Math.abs(kEncoderFrontRightRotation.getAbsolutePosition() - getMotorRotation(mRotation, RotationEncoderOffset.kEncoderFrontRightOffset, kEncoderFrontRightRotation.getAbsolutePosition())) > mError) {
          kMotorFrontRightRotation.set(mTurnSpeed * getSign(kEncoderFrontRightRotation.getAbsolutePosition(), getMotorRotation(mRotation, RotationEncoderOffset.kEncoderFrontRightOffset, kEncoderFrontRightRotation.getAbsolutePosition()), 0));
        }
        else {
          kMotorFrontRightRotation.set(0);
        }
        if (Math.abs(kEncoderBackRightRotation.getAbsolutePosition() - getMotorRotation(mRotation, RotationEncoderOffset.kEncoderBackRightOffset, kEncoderBackRightRotation.getAbsolutePosition())) > mError) {
          kMotorBackRightRotation.set(mTurnSpeed * getSign(kEncoderBackRightRotation.getAbsolutePosition(), getMotorRotation(mRotation, RotationEncoderOffset.kEncoderBackRightOffset, kEncoderBackRightRotation.getAbsolutePosition()), 1));
        }
        else {
          kMotorBackRightRotation.set(0);
        }
        if (Math.abs(kEncoderFrontLeftRotation.getAbsolutePosition() - getMotorRotation(mRotation, RotationEncoderOffset.kEncoderFrontLeftOffset, kEncoderFrontLeftRotation.getAbsolutePosition())) > mError) {
          kMotorFrontLeftRotation.set(mTurnSpeed * getSign(kEncoderBackRightRotation.getAbsolutePosition(), getMotorRotation(mRotation, RotationEncoderOffset.kEncoderBackRightOffset, kEncoderFrontLeftRotation.getAbsolutePosition()), 2));
        }
        else {
          kMotorFrontLeftRotation.set(0);
        }
        if (Math.abs(kEncoderBackLeftRotation.getAbsolutePosition() - getMotorRotation(mRotation, RotationEncoderOffset.kEncoderBackLeftOffset, kEncoderBackLeftRotation.getAbsolutePosition())) > mError) {
          kMotorBackLeftRotation.set(mTurnSpeed * getSign(kEncoderBackRightRotation.getAbsolutePosition(), getMotorRotation(mRotation, RotationEncoderOffset.kEncoderBackRightOffset, kEncoderBackLeftRotation.getAbsolutePosition()), 3));
        }
        else {
          kMotorBackLeftRotation.set(0);
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
        speed *= -mSlowDown;
        kMotorFrontRightDrive.set(-speed);
        kMotorBackRightDrive.set(-speed);
        kMotorFrontLeftDrive.set(-speed);
        kMotorBackLeftDrive.set(speed);
        mRotation = 0.25/2 + 0.25;
        if (Math.abs(kEncoderFrontRightRotation.getAbsolutePosition() - getMotorRotation(mRotation, RotationEncoderOffset.kEncoderFrontRightOffset, kEncoderFrontRightRotation.getAbsolutePosition())) > mError) {
          kMotorFrontRightRotation.set(mTurnSpeed * getSign(kEncoderFrontRightRotation.getAbsolutePosition(), getMotorRotation(mRotation, RotationEncoderOffset.kEncoderFrontRightOffset, kEncoderFrontRightRotation.getAbsolutePosition()), 0));
        }
        else {
          kMotorBackRightRotation.set(0);
        }
        mRotation = 0.25/2 + 0.50;
        if (Math.abs(kEncoderBackRightRotation.getAbsolutePosition() - getMotorRotation(mRotation, RotationEncoderOffset.kEncoderBackRightOffset, kEncoderBackRightRotation.getAbsolutePosition())) > mError) {
          kMotorBackRightRotation.set(mTurnSpeed * getSign(kEncoderBackRightRotation.getAbsolutePosition(), getMotorRotation(mRotation, RotationEncoderOffset.kEncoderBackRightOffset, kEncoderBackRightRotation.getAbsolutePosition()), 1));
        }
        else {
          kMotorBackRightRotation.set(0);
        }
        mRotation = 0.25/2;
        if (Math.abs(kEncoderFrontLeftRotation.getAbsolutePosition() - getMotorRotation(mRotation, RotationEncoderOffset.kEncoderFrontLeftOffset, kEncoderFrontLeftRotation.getAbsolutePosition())) > mError) {
          kMotorFrontLeftRotation.set(mTurnSpeed * getSign(kEncoderBackRightRotation.getAbsolutePosition(), getMotorRotation(mRotation, RotationEncoderOffset.kEncoderBackRightOffset, kEncoderFrontLeftRotation.getAbsolutePosition()), 1));
        }
        else {
          kMotorFrontLeftRotation.set(0);
        }
        mRotation = 0.25/2 + 0.75;
        if (Math.abs(kEncoderBackLeftRotation.getAbsolutePosition() - getMotorRotation(mRotation, RotationEncoderOffset.kEncoderBackLeftOffset, kEncoderBackLeftRotation.getAbsolutePosition())) > mError) {
          kMotorBackLeftRotation.set(mTurnSpeed * getSign(kEncoderBackRightRotation.getAbsolutePosition(), getMotorRotation(mRotation, RotationEncoderOffset.kEncoderBackRightOffset, kEncoderBackLeftRotation.getAbsolutePosition()), 1));
        }
        else {
          kMotorBackLeftRotation.set(0);
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

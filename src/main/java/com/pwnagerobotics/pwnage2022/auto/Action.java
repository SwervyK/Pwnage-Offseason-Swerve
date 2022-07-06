package com.pwnagerobotics.pwnage2022.auto;

import edu.wpi.first.wpilibj.Timer;

public class Action {

    private double kSpeed;
    private double kRobotAngle;
    private double kTurnSpeed;
    private boolean kFieldCentricRotation;
    private double kRotationAngle;

    public Action(double speed, double robotAngle, double turnSpeed) {
        kSpeed = speed;
        kRobotAngle = robotAngle;
        kTurnSpeed = turnSpeed;
        kFieldCentricRotation = false;
    }

    public Action(double throttle, double strafe, double rotationX, double rotationY, boolean fieldCentricRotation) {
        kRobotAngle = Math.toDegrees(Math.atan2(strafe, throttle));
        kRobotAngle = (kRobotAngle >= 0) ? kRobotAngle : kRobotAngle + 360;
        kSpeed = Math.sqrt(Math.pow(Math.abs(strafe), 2) + Math.pow(Math.abs(throttle), 2));
        kTurnSpeed = rotationX;
        kRotationAngle = Math.toDegrees(Math.atan2(rotationX, rotationY));
        kRotationAngle = (kRotationAngle >= 0) ? kRotationAngle : kRotationAngle + 360;
        kFieldCentricRotation = fieldCentricRotation;
    }

    public Action(double speed, double robotAngle, double rotationAngle, boolean fieldCentricRotation) {
        kSpeed = speed;
        kRobotAngle = robotAngle;
        kRotationAngle = rotationAngle;
        kFieldCentricRotation = true;
    }

    public double[] getDrive() {
        return new double[] {kSpeed*Math.sin(Math.toRadians(kRobotAngle)), kSpeed*Math.cos(Math.toRadians(kRobotAngle))};
    }

    public double[] getRotation() {
        if (kFieldCentricRotation) {
            return new double[] {Math.cos(Math.toRadians(kRotationAngle)), Math.sin(Math.toRadians(kRotationAngle))};
        } else {
            return new double[] {kTurnSpeed};
        }
    }

    public boolean getFieldCentricRotation() {
        return kFieldCentricRotation;
    }

    // Recorder
    private double kTimestamp;

    public void setTimestamp(double timestamp) {
        kTimestamp = timestamp;
    }

    public double getTimestamp() {
        return kTimestamp;
    }

    // Playback
    private double kDurration;
    private double kStartTime;

    public void startAction() {
        kStartTime = Timer.getFPGATimestamp();
    }

    public void setDurration(double durration) {
        kDurration = durration;
    }

    public boolean isDone() {
        return Timer.getFPGATimestamp() - kStartTime > kDurration;
    }
}

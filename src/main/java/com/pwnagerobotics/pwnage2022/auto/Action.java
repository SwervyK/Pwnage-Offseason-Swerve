package com.pwnagerobotics.pwnage2022.auto;

import edu.wpi.first.wpilibj.Timer;

public class Action {

    private double kMagnitude;
    private double kDirection;
    private double kRotationSpeed;
    private boolean kFieldCentricRotation;
    private double kRobotAngle;

    public Action(ControllerState controller, boolean fieldCentricRotation) {
        kMagnitude = controller.kMagnitude;
        kDirection = controller.kDirection;
        kRotationSpeed = controller.kRotationSpeed;
        kFieldCentricRotation = fieldCentricRotation;
        kRobotAngle = controller.kRobotAngle;
    }

    public Action(RobotState controller, boolean fieldCentricRotation) {
        kMagnitude = controller.kMagnitude;
        kDirection = controller.kDirection;
        kRotationSpeed = controller.kTurn;
        kFieldCentricRotation = fieldCentricRotation;
        kRobotAngle = controller.kTurn;
    }

    public Action(RobotState controller, boolean fieldCentricRotation, double duration) {
        kMagnitude = controller.kMagnitude;
        kDirection = controller.kDirection;
        kRotationSpeed = controller.kTurn;
        kFieldCentricRotation = fieldCentricRotation;
        kRobotAngle = controller.kTurn;
        kDuration = duration;
    }

    public static class ControllerState {
        public double kMagnitude;
        public double kDirection;
        public double kRotationSpeed;
        public double kRobotAngle;

        public ControllerState(double throttle, double strafe, double rotationX, double rotationY) {
            kDirection = Math.toDegrees(Math.atan2(strafe, throttle));
            if (kDirection < 0) kDirection += 360;
            kMagnitude = Math.sqrt(Math.pow(Math.abs(strafe), 2) + Math.pow(Math.abs(throttle), 2));
            kRotationSpeed = rotationX;
            kRobotAngle = Math.toDegrees(Math.atan2(rotationX, rotationY));
            if (kRobotAngle < 0) kRobotAngle += 360;
        }
    }

    public static class RobotState {
        public double kMagnitude;
        public double kDirection;
        public double kTurn; // Turn speed or angle depending on constructor called

        public RobotState(double magnitude, double direction, double turn) {
            kMagnitude = magnitude;
            kDirection = direction;
            kTurn = turn;
        }
    }

    public double[] getDrive() { // throttle and strafe
        return new double[] {kMagnitude*Math.sin(Math.toRadians(kDirection)), kMagnitude*Math.cos(Math.toRadians(kDirection))};
    }

    public double[] getRotation() { // rotationX and rotationY
        if (kFieldCentricRotation) {
            return new double[] {Math.cos(Math.toRadians(kRobotAngle)), Math.sin(Math.toRadians(kRobotAngle))};
        } else {
            return new double[] {kRotationSpeed, 0};
        }
    }

    public boolean isFieldCentricRotation() {
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
    private double kDuration;
    private double kStartTime;

    public void startAction() {
        kStartTime = Timer.getFPGATimestamp();
    }

    public void setDurration(double durration) {
        kDuration = durration;
    }

    public boolean isDone() {
        return Timer.getFPGATimestamp() - kStartTime > kDuration;
    }
}

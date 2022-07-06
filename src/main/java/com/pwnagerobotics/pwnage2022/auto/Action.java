package com.pwnagerobotics.pwnage2022.auto;

import edu.wpi.first.wpilibj.Timer;

public class Action {

    private double kSpeed;
    private double kRobotAngle;
    private double kTurnSpeed;
    private boolean kFieldCentricRotation;
    private double kRotationAngle;

    public Action(ControllerState controller, boolean fieldCentricRotation) {
        kSpeed = controller.kSpeed;
        kRobotAngle = controller.kRobotAngle;
        kTurnSpeed = controller.kTurnSpeed;
        kFieldCentricRotation = fieldCentricRotation;
        kRotationAngle = controller.kRotationAngle;
    }

    public Action(RobotState controller, boolean fieldCentricRotation) {
        kSpeed = controller.kSpeed;
        kRobotAngle = controller.kRobotAngle;
        kTurnSpeed = controller.kTurn;
        kFieldCentricRotation = fieldCentricRotation;
        kRotationAngle = controller.kTurn;
    }

    public Action(RobotState controller, boolean fieldCentricRotation, double durration) {
        kSpeed = controller.kSpeed;
        kRobotAngle = controller.kRobotAngle;
        kTurnSpeed = controller.kTurn;
        kFieldCentricRotation = fieldCentricRotation;
        kRotationAngle = controller.kTurn;
        kDurration = durration;
    }

    public static class ControllerState {
        public double kSpeed;
        public double kRobotAngle;
        public double kTurnSpeed;
        public double kRotationAngle;

        public ControllerState(double throttle, double strafe, double rotationX, double rotationY) {
            kRobotAngle = Math.toDegrees(Math.atan2(strafe, throttle));
            kRobotAngle = (kRobotAngle >= 0) ? kRobotAngle : kRobotAngle + 360;
            kSpeed = Math.sqrt(Math.pow(Math.abs(strafe), 2) + Math.pow(Math.abs(throttle), 2));
            kTurnSpeed = rotationX;
            kRotationAngle = Math.toDegrees(Math.atan2(rotationX, rotationY));
            kRotationAngle = (kRotationAngle >= 0) ? kRotationAngle : kRotationAngle + 360;
        }
    }

    public static class RobotState {
        public double kSpeed;
        public double kRobotAngle;
        public double kTurn; // Turn speed or angle

        public RobotState(double speed, double robotAngle, double turn) {
            kSpeed = speed;
            kRobotAngle = robotAngle;
            kTurn = turn;
        }
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

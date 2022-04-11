package com.pwnagerobotics.pwnage2022.lib;

public class SwerveModuleConstants {
    public String kName = "Name";
    public int kDriveId = 0;
    public int kRotationId = 0;
    public int kRotationEncoderId = 0;

    public double kp = 0.7;
    public double ki = 0.015;
    public double kd = 0.0;
    
    public double kRotationOffset = 0.0;
    public double kRotationError = 5; // Degrees
    public double kWheelDiameter = 0.0;
    public double kTurnDegree = 0.0; // TODO base this off center of rotation
}
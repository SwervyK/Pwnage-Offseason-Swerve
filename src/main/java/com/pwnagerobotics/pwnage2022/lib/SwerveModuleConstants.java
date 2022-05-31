package com.pwnagerobotics.pwnage2022.lib;

public class SwerveModuleConstants {
    public String kName = "Name";
    public int kDriveId = 0;
    public int kRotationId = 0;
    public int kDriveEncoderId = 0;
    public int kRotationEncoderId = 0;
    public int kPDPId = 0;

    public double kp = 0.01;
    public double ki = 0.001;
    public double kd = 0.0;
    
    public double kRotationOffset = 0.0;
    public double kRotationError = 3; // Degrees
    public double kWheelDiameter = 0.0;
}
package com.pwnagerobotics.pwnage2022.subsystems;

import java.util.Scanner;

import edu.wpi.first.math.controller.PIDController;

public class TestDrive {
    
    private static double rotaionEncoder = 0.0;
    private static Scanner s = new Scanner(System.in);
    private static PIDController pidController = new PIDController(0.7, 0.015, 0);
    public static void main(String[] args) {
        pidController.setIntegratorRange(0, 1);
        while(s.nextLine() != "d") {
            setModule(0.5);
        }
    }
    
    private static void setModule(double rotation)
    {
        // Postion
        double wantedPosition = rotation;
        if (wantedPosition > 1) wantedPosition -= 1;
        
        // Distance
        double distance = getDistance(rotaionEncoder, wantedPosition);
        if (distance > 0.25) {
            wantedPosition -= 0.5;
            if (wantedPosition < 0) wantedPosition += 1;
            distance -= 0.25;
        }
        
        double motorValue = 0;
        // motorValue = pidController.calculate(rotaionEncoder, wantedPosition);
        
        if (Math.abs(rotaionEncoder - wantedPosition) > 0.08) {
            motorValue = 1 * Math.signum(distance);
        }
        else {
            motorValue = 0;
        }
        if (Math.abs(motorValue) > 0.05) rotaionEncoder += 0.05;
        System.out.println("Motor: " + motorValue + "|Encoder: " + rotaionEncoder);
    }
    
    private static double getDistance(double encoder, double controller) {
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
}

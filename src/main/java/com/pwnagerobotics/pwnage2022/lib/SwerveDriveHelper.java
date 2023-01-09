package com.pwnagerobotics.pwnage2022.lib;

import com.pwnagerobotics.pwnage2022.Constants;
import com.pwnagerobotics.pwnage2022.humans.driver.XboxDriver;
import com.pwnagerobotics.pwnage2022.subsystems.Drive.DriveMode;

public class SwerveDriveHelper {

    private SwerveDriveHelper() { }
    
    // Wraps a value proportionally between min and max
    // (EX: max = 360, min = 0, value = 361, wrapAround = true, result = 1)
    // (EX: max = 360, min = 0, value = 361, wrapAround = false, result = 360)
    public static double clamp(double value, double max, double min, boolean wrapAround) {
        if (wrapAround) {
            if (value > max)
                return (value - (max-min) * ((int)((value-max-1)/(max - min)))) - max + min;
            else if (value < min)
                return value + (max-min) * -((((int)((value-max)/(max - min))))+1) - min + max;
            return value;
        }
        else 
        return (value>=max)?max:(value<=min)?min:value;
    }

    // Get shortest distance between 2 points
    // Pass in a value between 0 and maxAngle
    // Returns a value between 0 and maxAngle/2
    public static double getAngularDistance(double current, double wanted, double maxAngle) {
        double difference = wanted - current;
        if (Math.abs(difference) > maxAngle/2) difference += maxAngle * -Math.signum(difference);
            return difference;
    }

    public static double[] jukeMove(boolean jukeRight, boolean jukeLeft, double direction, double centerX, double centerY) {
        int side = (int)(nearestPoleSnapRadians(direction, Math.PI/4)/Math.PI/2);
        if (jukeRight && centerX == 0 && centerY == 0) {
            centerX = Constants.kDriveWidth*(side<2?1:-1);
            centerY = Constants.kDriveLength*(side==0||side==3?1:-1);
        }
        else if (jukeLeft && centerX == 0 && centerY == 0) {
          if (++side > 3) side -= 3;
          centerX = Constants.kDriveWidth*(side<2?-1:1);
          centerY = Constants.kDriveLength*(side==1||side==3?-1:1);
        }
        else if (!jukeRight && !jukeLeft ) {
            centerX = 0;
            centerY = 0;
        }

        return new double[] { centerX, centerY };
    }

    // Get nearest pole of an angle (N S E W)
    private static double nearestPoleSnapRadians(double angle, double threshold) {
        double poleSin = 0.0;
        double poleCos = 0.0;
        double sin = Math.sin(angle);
        double cos = Math.cos(angle);
        if (Math.abs(cos) > Math.abs(sin)) {
            poleCos = Math.signum(cos);
            poleSin = 0.0;
        } 
        else {
            poleCos = 0.0;
            poleSin = Math.signum(sin);
        }
        double pole = Math.atan2(poleSin, poleCos);
        if (Math.abs(pole - angle) <= threshold) {
            return pole;
        }
        else {
            return angle;
        }
    }

    // direction, magnitude
    public static double[] applyControlEffects(double throttle, double strafe, double rotationX, double rotationY, DriveMode mode, double gyroRadians) {
        // TODO use vectors
        double direction = Math.atan2(strafe, throttle); // Find what angle we want to drive at
        double magnitude = Math.hypot(Math.abs(strafe), Math.abs(throttle)); // Get wanted speed of robot
        magnitude = XboxDriver.scaleController(SwerveDriveHelper.clamp(magnitude, 1, 0, false), Constants.kDriveMaxValue, Constants.kDriveMinValue);

        // Pole Snapping
        if (magnitude > Constants.kPoleSnappingThreshold) direction = nearestPoleSnapRadians(direction-((mode == DriveMode.FIELD)?0:gyroRadians), Constants.kPoleSnappingAngle);
        return new double[] {direction, magnitude};
    }

    // throttle, strafe, rotation
    public static double[] convertControlEffects(double magnitude, double angle, double rotation) {
        angle = Math.toRadians(angle);
        return new double[] { magnitude * Math.cos(angle), magnitude * Math.sin(angle), rotation };
    }
}
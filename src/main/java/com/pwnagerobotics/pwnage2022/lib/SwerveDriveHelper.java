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
    // Pass in a value between 0 and 360
    // Returns a value between 0 and 180
    public static double getDistance(double current, double wanted) {
        double difference = wanted - current;
        if (Math.abs(difference) > 180) difference += 360 * -Math.signum(difference);
            return difference;
    }

    // Get module rotation angles using x and y position relative to center of robot
    // EX: on a square robot everything is 45 degrees
    public static double getTurnAngle(double xPos, double yPos) {
        double theta = Math.toDegrees(Math.atan2(yPos, xPos));
        if (theta > 0) theta -= 360;
        return Math.abs(theta);
    }

    public static double[] jukeMove(boolean jukeRight, boolean jukeLeft, double direction, double centerX, double centerY) {
        if (direction < 0) direction += 360;
        int side = (int)(nearestPoleSnap(direction, 45)/90);
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
    private static double nearestPoleSnap(double angle, double threshold) {
        double poleSin = 0.0;
        double poleCos = 0.0;
        double sin = Math.sin(Math.toRadians(angle));
        double cos = Math.cos(Math.toRadians(angle));
        if (Math.abs(cos) > Math.abs(sin)) {
        poleCos = Math.signum(cos);
        poleSin = 0.0;
        } 
        else {
        poleCos = 0.0;
        poleSin = Math.signum(sin);
        }
        double pole = Math.atan2(poleSin, poleCos);
        if (pole < 0) pole += 2*Math.PI;
        if (Math.toRadians(angle) > Math.PI && poleCos == 1) pole = 2*Math.PI;
        if (Math.abs(pole - Math.toRadians(angle)) <= Math.toRadians(threshold)) {
        double result = Math.toDegrees(Math.atan2(poleSin, poleCos));
        if (result < 0) result += 360;
        return result;
        }
        else 
        return angle;
    }

    // direction, magnitude
    public static double[] applyControlEffects(double throttle, double strafe, double rotationX, double rotationY, DriveMode mode, double gyroDegrees) {
        double direction = Math.toDegrees(Math.atan2(strafe, throttle)); // Find what angle we want to drive at // TODO -180 to 180
        if (direction < 0) direction += 360; // Convert from (-180 to 180) to (0 to 360)
        double magnitude = Math.hypot(Math.abs(strafe), Math.abs(throttle)); // Get wanted speed of robot
        magnitude = XboxDriver.scaleController(SwerveDriveHelper.clamp(magnitude, 1, 0, false), Constants.kDriveMaxValue, Constants.kDriveMinValue);

        // Pole Snapping
        if (magnitude > Constants.kPoleSnappingThreshold) direction = nearestPoleSnap(direction-((mode == DriveMode.FIELD)?0:gyroDegrees), Constants.kPoleSnappingAngle);
        return new double[] {direction, magnitude};
    }

    // throttle, strafe, rotation
    public static double[] convertControlEffects(double magnitude, double angle, double rotation) {
        angle = Math.toRadians(angle);
        return new double[] { magnitude * Math.sin(angle), magnitude * Math.cos(angle), rotation };
    }
}
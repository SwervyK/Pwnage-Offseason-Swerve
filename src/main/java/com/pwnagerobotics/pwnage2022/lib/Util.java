package com.pwnagerobotics.pwnage2022.lib;

public class Util {

    // Wraps a value proportionally between min and max
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
        double difference = current - wanted;
        if (Math.abs(difference) > 180) difference += 360 * -Math.signum(difference);
        return difference;
    }
}

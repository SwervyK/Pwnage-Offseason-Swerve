package com.pwnagerobotics.pwnage2022.lib;

public class Vector {

    private double x = 0;
    private double y = 0;
    private double mMagnitude = 0;
    private double mAngle = 0; // Rad

    public Vector(double x, double y) {
        this.x = x;
        this.y = y;
        recalculate();
    }

    private void recalculate() {
        mMagnitude = Math.hypot(x, y);
        mAngle = Math.atan2(y, x);
        if (mAngle < 0) mAngle += 2*Math.PI;
    }

    public void scaleVector(double scaler) {
        mMagnitude *= scaler;
    }

    public void addVector(Vector v) {
        x += v.getX();
        y += v.getY();
        recalculate();
    }

    public double nearestPole(double threshold) {
        double poleSin = 0.0;
        double poleCos = 0.0;
        double sin = Math.sin(mAngle);
        double cos = Math.cos(mAngle);
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
        if (mAngle > Math.PI && poleCos == 1) pole = 2*Math.PI;
        if (Math.abs(pole - mAngle) <= Math.toRadians(threshold)) {
          double result = Math.toDegrees(Math.atan2(poleSin, poleCos));
          if (result < 0) result += 360;
          return result;
        }
        else 
          return mAngle;
    }

    public double getMagnitude() {
        return mMagnitude;
    }

    public double getAngle() {
        return Math.toDegrees(mAngle);
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getAdjustedAngle() {
        double angle = getAngle();
        if (x == 0) angle = 0;
        if (x < 0) angle += 180;
        else if (y < 0) angle += 360;
        return angle;
    }

    public static double getAdjustedAngle(double x, double y) {
        double angle = Math.toDegrees(Math.atan(y / x));
        if (x == 0) angle = 0;
        if (x < 0) angle += 180;
        else if (y < 0) angle += 360;
        return angle;
    }

    public static double getMagnitude(double x, double y) {
        return Math.hypot(x, y);
    }
}

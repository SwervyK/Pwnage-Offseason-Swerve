package com.pwnagerobotics.pwnage2022.lib;

public class Vector {

    public double x = 0;
    public double y = 0;
    private double mMagnitude = 0;
    private double mAngle = 0; // Rad

    public static void main(String[] args) {
        Vector v = new Vector(0, 1);
        System.out.println(v.getAngleFromZero());
        v = new Vector(1, 1);
        System.out.println(v.getAngleFromZero());
        v = new Vector(1, 0);
        System.out.println(v.getAngleFromZero());
        v = new Vector(1, -1);
        System.out.println(v.getAngleFromZero());
        v = new Vector(0, -1);
        System.out.println(v.getAngleFromZero());
        v = new Vector(-1, -1);
        System.out.println(v.getAngleFromZero());
        v = new Vector(-1, 0);
        System.out.println(v.getAngleFromZero());
        v = new Vector(-1, 1);
        System.out.println(v.getAngleFromZero());
    }

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
        x += v.x;
        y += v.y;
        recalculate();
    }

    public double getMagnitude() {
        return mMagnitude;
    }

    public double getAngle() {
        return Math.toDegrees(mAngle);
    }
}

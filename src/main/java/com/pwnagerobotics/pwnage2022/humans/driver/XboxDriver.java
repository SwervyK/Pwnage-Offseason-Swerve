package com.pwnagerobotics.pwnage2022.humans.driver;

import com.pwnagerobotics.lib.drivers.XboxController;
import com.pwnagerobotics.lib.drivers.XboxController.Axis;
import com.pwnagerobotics.lib.drivers.XboxController.Button;
import com.pwnagerobotics.lib.drivers.XboxController.Side;

public class XboxDriver {
    private final double mDeadband = 0.1;
    private final XboxController mController = new XboxController(0);
    
    public double getPositionX() {
        return mController.getJoystick(Side.LEFT, Axis.X);
    }

    // private static double deadband(double value, double deadband) {
    //     if (Math.abs(value) > deadband) {
    //       if (value > 0.0) {
    //         return (value - deadband) / (1.0 - deadband);
    //       } else {
    //         return (value + deadband) / (1.0 - deadband);
    //       }
    //     } else {
    //       return 0.0;
    //     }
    //   }
    
    //   private static double modifyAxis(double value) {
    //     // Deadband
    //     value = deadband(value, 0.08);
    
    //     // Square the axis
    //     value = Math.copySign(value * value, value);
    
    //     return value;
    //   }

    public double getPositionY() {
        return mController.getJoystick(Side.LEFT, Axis.Y);
    }
    
    public double getRotationX() {
        return mController.getJoystick(Side.RIGHT, Axis.X);
    }

    public double getRotationY() {
        return mController.getJoystick(Side.RIGHT, Axis.Y);
    }

    public boolean wantFieldCentricDrive() {
        return mController.getTrigger(Side.LEFT);
    }

    public boolean wantFieldCentricRotation() {
        return mController.getTrigger(Side.RIGHT);
    }

    public int getDPad() {
        return mController.getDPad();
    }

    public boolean wantShift() {
        return mController.getTrigger(Side.LEFT);
    }

    private double handleDeadband(double value) {
        if (Math.abs(value) < mDeadband) {
            return 0;
        } else {
            return value;
        }
    }
}

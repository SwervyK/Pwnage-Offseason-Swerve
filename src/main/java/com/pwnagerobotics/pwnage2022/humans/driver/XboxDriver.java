package com.pwnagerobotics.pwnage2022.humans.driver;

import com.pwnagerobotics.lib.drivers.XboxController;
import com.pwnagerobotics.lib.drivers.XboxController.Axis;
import com.pwnagerobotics.lib.drivers.XboxController.Button;
import com.pwnagerobotics.lib.drivers.XboxController.Side;
import com.pwnagerobotics.pwnage2022.Constants;

public class XboxDriver {
    private final XboxController mController = new XboxController(0);
    
    public double getPositionX() {
        return modifyAxis(mController.getJoystick(Side.LEFT, Axis.X), getPositionY(true), Constants.kLeftStickDeadband);
    }

    public double getPositionY() {
        return modifyAxis(mController.getJoystick(Side.LEFT, Axis.Y), getPositionX(true), Constants.kLeftStickDeadband);
    }
    
    public double getRotationX() {
        return modifyAxis(mController.getJoystick(Side.RIGHT, Axis.X), getRotationY(true), Constants.kRightStickDeadband);
    }

    public double getRotationY() {
        return modifyAxis(mController.getJoystick(Side.RIGHT, Axis.Y), getRotationX(true), Constants.kRightStickDeadband);
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

    public double getPositionX(boolean raw) {
        return mController.getJoystick(Side.LEFT, Axis.X);
    }

    public double getPositionY(boolean raw) {
        return mController.getJoystick(Side.LEFT, Axis.Y);
    }
    
    public double getRotationX(boolean raw) {
        return mController.getJoystick(Side.RIGHT, Axis.X);
    }

    public double getRotationY(boolean raw) {
        return mController.getJoystick(Side.RIGHT, Axis.Y);
    }

    private static double deadband(double value, double opposite, double deadband) {
        if (Math.abs(value) < deadband) {
            if (Math.abs(opposite) > deadband) {
                return value;
            }
            else {
                return 0;
            }
        }
        else {
            return value;
        }
        // if (Math.abs(value) > deadband) { Re-Scale so you get a full 0-1
        //   if (value > 0.0) {
        //     return (value - deadband) / (1.0 - deadband);
        //   } else {
        //     return (value + deadband) / (1.0 - deadband);
        //   }
        // } else {
        //   return 0.0;
        // }
      }

      public static double scaleController(double value, double minValue) {
        return (value * (1-minValue)) + minValue;
      }
    
      private static double modifyAxis(double value, double opposite, double deadband) {
        // Deadband
        value = deadband(value, opposite, deadband);
    
        // Square the axis
        value = Math.copySign(value * value, value);
    
        return value;
      }
}

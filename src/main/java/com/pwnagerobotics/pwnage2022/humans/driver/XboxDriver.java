package com.pwnagerobotics.pwnage2022.humans.driver;

import com.pwnagerobotics.lib.drivers.XboxController;
import com.pwnagerobotics.lib.drivers.XboxController.Axis;
import com.pwnagerobotics.lib.drivers.XboxController.Button;
import com.pwnagerobotics.lib.drivers.XboxController.Side;

public class XboxDriver {
    private final double mDeadband = 0.1;
    private final XboxController mController = new XboxController(0);
    
    public double getPositionX() {
        return handleDeadband(mController.getJoystick(Side.LEFT, Axis.X));
    }

    public double getPositionY() {
        return handleDeadband(mController.getJoystick(Side.LEFT, Axis.Y));
    }
    
    public double getRotationX() {
        return handleDeadband(mController.getJoystick(Side.RIGHT, Axis.X));
    }

    public double getRotationY() {
        return handleDeadband(mController.getJoystick(Side.RIGHT, Axis.Y));
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

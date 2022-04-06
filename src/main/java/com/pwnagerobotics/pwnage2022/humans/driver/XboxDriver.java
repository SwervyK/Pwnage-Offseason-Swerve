package com.pwnagerobotics.pwnage2022.humans.driver;

import com.pwnagerobotics.lib.drivers.XboxController;
import com.pwnagerobotics.lib.drivers.XboxController.Axis;
import com.pwnagerobotics.lib.drivers.XboxController.Button;
import com.pwnagerobotics.lib.drivers.XboxController.Side;
import com.team254.lib.util.Deadband;

public class XboxDriver {
    private final double mDeadband = 0.1;
    private final XboxController mController = new XboxController(0);
    
    public double getPositionX() {
        return mController.getJoystick(Side.LEFT, Axis.X);
    }

    public double getPositionY() {
        return mController.getJoystick(Side.LEFT, Axis.Y);
    }
    
    public double getRotationX() {
        return mController.getJoystick(Side.RIGHT, Axis.X);
    }

    public double getRotationY() {
        return mController.getJoystick(Side.RIGHT, Axis.Y);
    }

    public int getDPad() {
        return mController.getDPad();
    }

    public boolean wantShift() {
        return mController.getTrigger(Side.LEFT);
    }
}

package com.pwnagerobotics.pwnage2022.humans.driver;

import com.pwnagerobotics.lib.drivers.XboxController;
import com.pwnagerobotics.lib.drivers.XboxController.Axis;
import com.pwnagerobotics.lib.drivers.XboxController.Button;
import com.pwnagerobotics.lib.drivers.XboxController.Side;

public class XboxDriver {
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

    public double getRoationY() {
        return mController.getJoystick(Side.RIGHT, Axis.X);
    }
}

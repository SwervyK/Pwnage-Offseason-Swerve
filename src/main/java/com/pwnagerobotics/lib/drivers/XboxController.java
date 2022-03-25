package com.pwnagerobotics.lib.drivers;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Joystick;

public class XboxController {
    private final Joystick mController;
    private final double kDeadband = 0.05;
    private final double kTriggerDeadband = 0.175;

    public enum Side {
        LEFT, RIGHT
    }

    public enum Axis {
        X, Y
    }

    public enum Button {
        A(1), B(2), X(3), Y(4), LB(5), RB(6), BACK(7), START(8), L_JOYSTICK(9), R_JOYSTICK(10);

        public final int id;

        Button(int id) {
            this.id = id;
        }
    }

    public XboxController(int port) {
        mController = new Joystick(port);
    }

    public double getJoystick(Side side, Axis axis) {
        double deadband = kDeadband;

        boolean left = side == Side.LEFT;
        boolean y = axis == Axis.Y;
        // multiplies by -1 if y-axis (inverted normally)
        return handleDeadband((y ? -1 : 1) * mController.getRawAxis((left ? 0 : 4) + (y ? 1 : 0)), deadband);
    }

    public boolean getTrigger(Side side) {
        return mController.getRawAxis(side == Side.LEFT ? 2 : 3) > kTriggerDeadband;
    }

    public boolean getButton(Button button) {
        return mController.getRawButton(button.id);
    }

    public int getDPad() {
        return mController.getPOV();
    }

    public void setRumble(boolean on) {
        mController.setRumble(RumbleType.kRightRumble, on ? 1 : 0);
    }

    private double handleDeadband(double value, double deadband) {
        return (Math.abs(value) > Math.abs(deadband)) ? value : 0;
    }
}
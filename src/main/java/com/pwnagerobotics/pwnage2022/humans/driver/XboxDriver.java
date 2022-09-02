package com.pwnagerobotics.pwnage2022.humans.driver;

import com.pwnagerobotics.lib.drivers.XboxController;
import com.pwnagerobotics.lib.drivers.XboxController.Axis;
import com.pwnagerobotics.lib.drivers.XboxController.Button;
import com.pwnagerobotics.lib.drivers.XboxController.Side;
import com.pwnagerobotics.pwnage2022.Constants;

public class XboxDriver {
    
    private final XboxController mController = new XboxController(0);

    public double getPositionX() {
        return modifyAxis(mController.getJoystick(Side.LEFT, Axis.X), mController.getJoystick(Side.LEFT, Axis.Y), Constants.kLeftStickDeadband);
    }

    public double getPositionY() {
        return modifyAxis(mController.getJoystick(Side.LEFT, Axis.Y), mController.getJoystick(Side.LEFT, Axis.X), Constants.kLeftStickDeadband);
    }

    public double getRotationX() {
        return modifyAxis(mController.getJoystick(Side.RIGHT, Axis.X), mController.getJoystick(Side.RIGHT, Axis.Y), Constants.kRightStickDeadband);
    }

    public double getRotationY() {
        return modifyAxis(mController.getJoystick(Side.RIGHT, Axis.Y), mController.getJoystick(Side.RIGHT, Axis.X), Constants.kRightStickDeadband);
    }

    public boolean wantFieldCentricDrive() {
        return mController.getTrigger(Side.LEFT);
    }

    public boolean wantFieldCentricRotation() {
        return mController.getTrigger(Side.RIGHT);
    }

    public boolean wantJukeRight() {
        return mController.getButton(Button.RB);
    }

    public boolean wantJukeLeft() {
        return mController.getButton(Button.LB);
    }

    public int getDPad() {
        return mController.getDPad();
    }

    /**
    * Proportionally scales a controller between max and min
    * @param value Value to scale
    * @param max Maximum value it can be
    * @param min Minimum value it can be
    * @return Adjusted value
    */
    public static double scaleController(double value, double max, double min) {
        return ((max-min)*((Math.abs(value)-0)/(1-0))+min)*Math.signum(value);
        //return (b−a)x−min(x)max(x)−min(x)+a //Min + Max
        //return (value * (1-min)) + min; //Min only
        //return (value * max); //Max only
    }

    /**
    * Returns value if value is greater that deadband or opposite is greater that deadband and squares it
    * @param value Value to modify
    * @param opposite Opposite controller value
    * @param deadband Minimum value controller needs to read to be considered valid
    * @return Modified value
    */
    private static double modifyAxis(double value, double opposite, double deadband) {
        // Deadband
        if (Math.abs(value) < deadband && Math.abs(opposite) < deadband) {
            return 0;
        }
        value = ((Math.abs(value) * (1-deadband)) + deadband) * Math.signum(value);
        
        // Square the axis
        value = Math.copySign(value * value, value);
        
        return value;
    }
}

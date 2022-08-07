package com.pwnagerobotics.pwnage2022.humans.driver;

import com.pwnagerobotics.lib.drivers.XboxController;
import com.pwnagerobotics.lib.drivers.XboxController.Axis;
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
    
    public int getDPad() {
        return mController.getDPad();
    }
    
    public static double scaleController(double value, double max, double min) {
        return ((max-min)*((Math.abs(value)-0)/(1-0))+min)*Math.signum(value);
        //xnormalized=(b−a)x−min(x)max(x)−min(x)+a //Min + Max
        //return (value * (1-min)) + min; //Min only
        //return (value * max); //Max only
    }

    private static double modifyAxis(double value, double opposite, double deadband) {
        // Deadband
        value = deadband(value, opposite, deadband);
        
        // Square the axis
        value = Math.copySign(value * value, value);
        
        return value;
    }
    
    private static double deadband(double value, double opposite, double deadband) {
        if (Math.abs(value) < deadband && Math.abs(opposite) > deadband) {
            return 0;
        }
        return ((Math.abs(value) * (1-deadband)) + deadband) * Math.signum(value);
    }
}

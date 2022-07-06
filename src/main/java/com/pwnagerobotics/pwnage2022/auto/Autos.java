package com.pwnagerobotics.pwnage2022.auto;

import com.pwnagerobotics.pwnage2022.auto.Action.RobotState;

public class Autos {
    
    public static Action[] square() {
        double driveSpeed = 0.5;
        double turnSpeed = 0;
        return new Action[] {
            new Action(new RobotState(driveSpeed, 0, turnSpeed), false, 1),
            new Action(new RobotState(driveSpeed, 90, turnSpeed), false, 1),
            new Action(new RobotState(driveSpeed, 180, turnSpeed), false, 1),
            new Action(new RobotState(driveSpeed, 270, turnSpeed), false, 1),
        };
    }

    public static Action[] squareFC() {
        double driveSpeed = 0.5;
        return new Action[] {
            new Action(new RobotState(driveSpeed, 0, 0), true, 1),
            new Action(new RobotState(driveSpeed, 90, 90), true, 1),
            new Action(new RobotState(driveSpeed, 180, 180), true, 1),
            new Action(new RobotState(driveSpeed, 270, 270), true, 1),
        };
    }

    public static Action[] spinLine() {
        double driveSpeed = 0.3;
        double turnSpeed = 0.7;
        return new Action[] {
            new Action(new RobotState(driveSpeed, 0, turnSpeed), false, 2),
            new Action(new RobotState(driveSpeed, 90, turnSpeed), false, 2),
        };
    }

    public static Action[] spinLineFC() {
        double driveSpeed = 0.3;
        return new Action[] {
            new Action(new RobotState(driveSpeed, 0, 0), true, 2),
            new Action(new RobotState(driveSpeed, 90, 90), true, 2),
        };
    }
}

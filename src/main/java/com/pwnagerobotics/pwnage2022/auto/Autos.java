package com.pwnagerobotics.pwnage2022.auto;

public class Autos {
    
    public static Action[] square() {
        return new Action[] {
            new Action(0.5, 0, 0),
            new Action(0.5, 90, 0),
            new Action(0.5, 180, 0),
            new Action(0.5, 270, 0),
            new Action(0.5, 0, 0),
        };
    }
}

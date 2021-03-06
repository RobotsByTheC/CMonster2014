/* 
 * Copyright (c) 2014 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package edu.wpi.first.wpilibj.buttons;

import edu.wpi.first.wpilibj.GenericHID;

/**
 *
 * @author Ben Wolsieffer
 */
public class JoystickAxisButton extends Button {

    private final GenericHID joystick;
    private final int axis;
    private final double value;

    public JoystickAxisButton(GenericHID joystick, int axis, double value) {
        this.joystick = joystick;
        this.axis = axis;
        this.value = value;
    }

    public boolean get() {
        double axisValue = joystick.getRawAxis(axis);
        return value < 0 ? axisValue < value : axisValue > value;
    }

}

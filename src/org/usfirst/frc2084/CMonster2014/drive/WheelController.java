/* 
 * Copyright (c) 2014 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc2084.CMonster2014.drive;

import edu.wpi.first.wpilibj.SpeedController;

/**
 * Controls a single wheel with an arbitrary number of motors. In many cases
 * this will actually control a single wheel, but in designs where two wheels
 * are connected to the same gearbox (such as most tank drive systems), it will
 * actually control two wheels.
 *
 * @author Ben Wolsieffer
 */
public class WheelController {

    protected SpeedController[] motors;
    private double inverted = 1;

    /**
     * Creates a new {@link WheelController} with one motor.
     *
     * @param motor the {@link SpeedController} that powers the wheel
     */
    public WheelController(SpeedController motor) {
        this(new SpeedController[]{motor});
    }

    /**
     * Creates a new {@link WheelController} with two motors.
     *
     * @param motor the {@link SpeedController} that powers the wheel
     * @param motor2 the second {@link SpeedController} that powers the wheel
     */
    public WheelController(SpeedController motor, SpeedController motor2) {
        this(new SpeedController[]{motor, motor2});
    }

    /**
     * Creates a new {@link WheelController} with three motors.
     *
     * @param motor the {@link SpeedController} that powers the wheel
     * @param motor2 the second {@link SpeedController} that powers the wheel
     * @param motor3 the third {@link SpeedController} that powers the wheel
     */
    public WheelController(SpeedController motor, SpeedController motor2, SpeedController motor3) {
        this(new SpeedController[]{motor, motor2, motor3});
    }

    /**
     * Creates a new {@link WheelController} with an arbitrary number of motors.
     * This constructor is only necessary if one wheel has more than 3 motors
     * (which seems unlikely), otherwise use the 1, 2, or 3 parameter versions
     * of the constructor.
     *
     * @param motors the array of {@link SpeedController} that powers the wheel
     */
    public WheelController(SpeedController[] motors) {
        this.motors = motors;
    }

    /**
     * Sets the speed of the wheel. For a basic wheel this just sets the power
     * to the motor but this class can be extended for more complex
     * functionality.
     *
     * @param speed the wheel speed between 1.0 and -1.0
     */
    public void set(double speed) {
        speed = DriveUtils.limit(speed) * inverted;
        for (int i = 0; i < motors.length; i++) {
            motors[i].set(speed);
        }
    }

    /**
     * Sets whether this wheel should be inverted.
     *
     * @param inverted whether the wheel is inverted
     */
    public void setInverted(boolean inverted) {
        this.inverted = inverted ? -1 : 1;
    }

    /**
     * Gets whether or not the wheel is inverted.
     *
     * @return the inverted state
     */
    public boolean isInverted() {
        return inverted == -1;
    }
}

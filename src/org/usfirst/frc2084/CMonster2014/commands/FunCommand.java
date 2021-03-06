/* 
 * Copyright (c) 2014 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc2084.CMonster2014.commands;

import edu.wpi.first.wpilibj.ADXL345_I2C;
import edu.wpi.first.wpilibj.DriverStationLCD;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc2084.CMonster2014.Robot;
import org.usfirst.frc2084.CMonster2014.RobotMap;
import org.usfirst.frc2084.CMonster2014.TargetTrackingCommunication;

/**
 * Command that is always running, calculates pi and report debugging values.
 *
 * @author Ben Wolsieffer
 */
public class FunCommand extends Command {

    /**
     * Stores the current value of pi. Used by the pi calculating algorithm.
     */
    private double pi;
    /**
     * Stores the value that is 1/4 of pi. Used by the pi calculating algorithm.
     * This is the actual value that the algorithm calculates.
     */
    private double pi4;
    /**
     * This value gets toggled each time the pi calculating algorithm runs. Used
     * by
     */
    private boolean piPositive = true;
    private long piIndex = 1;

    public FunCommand() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
        // Make sure this command still reports data and caclualtes pi when the 
        // robot is disabled.
        setRunWhenDisabled(true);
    }

    /**
     * Does nothing.
     */
    protected void initialize() {
    }

    /**
     * Calculates Pi :), and prints out debugging data to the SmartDashboard.
     */
    protected void execute() {
        // Calculate Pi!!!! (http://en.wikipedia.org/wiki/Leibniz_formula_for_%CF%80)
        // This method is really slow
        pi4 += (piPositive ? 1.0 : -1.0) / piIndex;
        piPositive = !piPositive;
        piIndex += 2.0;
        pi = pi4 * 4.0;
        // Print out Pi to the DS LCD (not the SmartDashboard) because it can 
        // display more digits
        DriverStationLCD.getInstance().println(DriverStationLCD.Line.kUser1, 1, "Pi: " + pi);
        DriverStationLCD.getInstance().updateLCD();
        // Pi
        // Report misc values
        SmartDashboard.putNumber("Gyro Rate", RobotMap.driveSubsystemSteeringGyro.getRate());
        SmartDashboard.putNumber("Gyro Angle", RobotMap.driveSubsystemSteeringGyro.getAngle());
        ADXL345_I2C.AllAxes accel = RobotMap.driveSubsystemAccelerometer.getAccelerations();
        SmartDashboard.putNumber("X Acceleration", accel.XAxis);
        SmartDashboard.putNumber("Y Acceleration", accel.YAxis);
        String targetState;
        switch (TargetTrackingCommunication.getState().value) {
            case TargetTrackingCommunication.State.HOT_VALUE:
                targetState = "Hot";
                break;
            case TargetTrackingCommunication.State.NOT_HOT_VALUE:
                targetState = "Not hot";
                break;
            default:
            case TargetTrackingCommunication.State.UNKNOWN_VALUE:
                targetState = "Unknown";
                break;
        }
        SmartDashboard.putString("Goal State", targetState);

        SmartDashboard.putNumber("Encoder Distance", RobotMap.driveSubsystemRearRightEncoder.getDistance());
        SmartDashboard.putNumber("Temperature", RobotMap.driveSubsystemSteeringGyroTemp.getTemp());
        
        Robot.ledSubsystem.updatePattern();
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}

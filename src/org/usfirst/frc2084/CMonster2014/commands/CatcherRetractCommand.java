/* 
 * Copyright (c) 2014 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc2084.CMonster2014.commands;

import org.usfirst.frc2084.CMonster2014.Robot;

/**
 * Command that closes the catcher. It takes a certain amount of time for the
 * pneumatics to move so this command extends {@link TimedCommand} which keeps
 * it from ending immediately.
 *
 * @author Ben Wolsieffer
 */
public class CatcherRetractCommand extends TimedCommand {

    public CatcherRetractCommand() {
        // I estimated that it takes around 0.25 seconds to close the catcher, so
        // that's how long this command takes.
        super(0.25);
        // This command uses the sweeper assembly so it requires the sweeper 
        // subsystem.
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
        requires(Robot.catcherSubsytem);
        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
    }

    /**
     * Closes the catcher.
     */
    protected void run() {
        Robot.catcherSubsytem.retract();
    }

    /**
     * Does nothing.
     */
    protected void end() {
    }

    /**
     * Does nothing.
     */
    protected void interrupted() {
    }
}

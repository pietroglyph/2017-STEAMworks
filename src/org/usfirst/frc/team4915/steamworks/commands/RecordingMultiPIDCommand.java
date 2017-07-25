package org.usfirst.frc.team4915.steamworks.commands;

import org.usfirst.frc.team4915.steamworks.subsystems.Drivetrain;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class RecordingMultiPIDCommand extends Command {
	
	private final Drivetrain m_drivetrain;
	private final boolean m_state;
	
    public RecordingMultiPIDCommand(Drivetrain drivetrain, boolean state) {
        m_drivetrain = drivetrain;
        m_state = state;
    	requires(m_drivetrain);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if (m_state) {
    		
    	} else {
    		
    	}
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

package org.usfirst.frc.team4915.steamworks.commands;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;

import org.usfirst.frc.team4915.steamworks.ReplayDataPoint;
import org.usfirst.frc.team4915.steamworks.subsystems.Drivetrain;

import com.ctre.CANTalon.TalonControlMode;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.command.Command;

/**
 * DriveMultiPIDCommand defines a PID for the port and starboard motor
 * which are driven by multiple set points.
 */
public class ReplayMultiPIDCommand extends Command {
	
    private final Drivetrain m_drivetrain;
    private PIDController m_portPIDController;
    private PIDController m_starboardPIDController;
    private ArrayList<ReplayDataPoint> m_replayData;
    private int m_curReplayIndex = 0;
    // We are using System.nanoTime as our monotonic time source everywhere,
    // it has no relation to wall time. Thus, it is unaffected by leap seconds
    // NTP updates and the like, but needs a start time to be meaningful.
    // It's also very accurate
    private long m_startTime;
	
    // Left motor
    private static final double p_P = 0, p_I = 0, p_D = 0, p_F = 0;
    //Right motor
    private static final double s_P = 0, s_I = 0, s_D = 0, s_F = 0;
    
    // FIXME: I have no idea what an encoder velocity is like (See CANTalon.getEncVelocity()), so I don't know what this should be
    private static final double INPUT_RANGE = 4915;
    private static final double ABSOLUTE_TOLERENCE = 4915;
    
    public ReplayMultiPIDCommand(Drivetrain drivetrain) {
    	m_drivetrain = drivetrain;
    	
    	SidePID portPID = new SidePID(m_drivetrain, Drivetrain.MotorSide.Port);
    	SidePID starboardPID = new SidePID(m_drivetrain, Drivetrain.MotorSide.Starboard);
    	
    	m_portPIDController = new PIDController(p_P, p_I, p_D, p_F, portPID, portPID);
        m_portPIDController.setOutputRange(-1, 1); // Set the output range so that this works with CANTalon.set() in PercentVBus mode
        // FIXME: Once again, I don't know the input range of CANTalon.getEncVelocity
        m_portPIDController.setInputRange(-INPUT_RANGE, INPUT_RANGE); // We limit our input range to revolutions, either direction
        m_portPIDController.setAbsoluteTolerance(ABSOLUTE_TOLERENCE); // This is the tolerance for error for reaching our target, targeting one inch
    	
        m_starboardPIDController = new PIDController(s_P, s_I, s_D, s_F, starboardPID, starboardPID);
        m_starboardPIDController.setOutputRange(-1, 1); // Set the output range so that this works with CANTalon.set() in PercentVBus mode
        // FIXME: Once again, I don't know the input range of CANTalon.getEncVelocity
        m_starboardPIDController.setInputRange(-INPUT_RANGE, INPUT_RANGE); // We limit our input range to revolutions, either direction
        m_starboardPIDController.setAbsoluteTolerance(ABSOLUTE_TOLERENCE); // This is the tolerance for error for reaching our target, targeting one inch
    	
    	requires(m_drivetrain);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        m_drivetrain.m_logger.info("DriveMultiPIDCommand initalize");
        m_drivetrain.setControlMode(TalonControlMode.PercentVbus, 12.0, -12.0,
                0.0, 0, 0, 0 /* zero PIDF */, 1 /* RobotDrive MaxOutput */);
        m_drivetrain.resetPosition();
        
        m_portPIDController.reset();
        m_starboardPIDController.reset();
        
        // This can't be done in the constructor because the constructor is called early enough that the lists are empty
        m_drivetrain.loadReplay();
        m_replayData = m_drivetrain.getReplayData();
        // Sort m_replayData by time
    	Collections.sort(m_replayData, new Comparator<ReplayDataPoint>() {
             @Override
             public int compare(ReplayDataPoint lhs, ReplayDataPoint rhs) {
                 return lhs.timeIndex > rhs.timeIndex ? -1 : (lhs.timeIndex < rhs.timeIndex ) ? 1 : 0;
             }
         });
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	updatePID(Drivetrain.MotorSide.Port);
    	updatePID(Drivetrain.MotorSide.Starboard);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	if (m_curReplayIndex > m_replayData.size()-1) {
    		m_drivetrain.m_logger.notice("ReplayMultiPID finished replaying.");
			return true;
		}
    	return false;
    }

    // Called once after isFinished returns true
    protected void end() {
        if (m_portPIDController.isEnabled())
        {
            m_portPIDController.reset();
            assert (!m_portPIDController.isEnabled());
        }
        if (m_starboardPIDController.isEnabled())
        {
            m_starboardPIDController.reset();
            assert (!m_starboardPIDController.isEnabled());
        }
        m_drivetrain.stop();
        m_drivetrain.m_logger.info("DriveMultiPIDCommand end");
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
        m_drivetrain.m_logger.info("DriveMultiPIDCommand interrupted");
    }
    
    private void updatePID(Drivetrain.MotorSide side) {
    	PIDController controller;
    	switch (side) {
    	case Port:
    		controller = m_portPIDController;
    		break;
    	case Starboard:
    		controller = m_starboardPIDController;
    		break;
    	default:
    		m_drivetrain.m_logger.error("updatePID unsupported motor side.");
    		return;
    	}
    	// If the controller is enabled then check to see if we're
    	// done with the current data point, if so set the target to 
    	// the next one. If the controller isn't enabled, set everything up
    	// (this resets everything too) including replay index and start time
    	if (controller.isEnabled()) {
    		if (m_startTime - System.nanoTime() >= m_replayData.get(m_curReplayIndex+1).timeIndex) {
    			m_curReplayIndex++;
    			if (!controller.onTarget()) {
    				m_drivetrain.m_logger.warning(side.name()+" PID controller is not on target in time.");
    			}
    			if (m_curReplayIndex > m_replayData.size()-1) {
    				return;
    			}
    	    	switch (side) {
    	    	case Port:
    	    		controller.setSetpoint(m_replayData.get(m_curReplayIndex).portVelocity);
    	    		break;
    	    	case Starboard:
    	    		controller.setSetpoint(m_replayData.get(m_curReplayIndex).starboardVelocity);
    	    		break;
    	    	} // We don't need a default here because the previous switch would have defaulted
    		}
    	} else {
    		m_curReplayIndex = 0;
    		switch (side) {
	    	case Port:
	    		controller.setSetpoint(m_replayData.get(m_curReplayIndex).portVelocity);
	    		break;
	    	case Starboard:
	    		controller.setSetpoint(m_replayData.get(m_curReplayIndex).starboardVelocity);
	    		break;
	    	} // We also don't need a default here
    		m_startTime = System.nanoTime();
    		controller.enable();
    	}
    }

}

class SidePID implements PIDSource, PIDOutput {
	private final Drivetrain m_drivetrain;
	private final Drivetrain.MotorSide m_side;
	
	public SidePID(Drivetrain drivetrain, Drivetrain.MotorSide side) {
		m_drivetrain = drivetrain;
		m_side = side;
	}
	
	@Override
	public void pidWrite(double output) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void setPIDSourceType(PIDSourceType pidSource) {
        if (pidSource != PIDSourceType.kDisplacement)
        {
            m_drivetrain.m_logger.error("SidePID only supports kDisplacement");
        }
	}

	@Override
	public PIDSourceType getPIDSourceType() {
		return PIDSourceType.kDisplacement;
	}

	@Override
	public double pidGet() {
		return m_drivetrain.getEncVelocity(m_side); // There is no documentation anywhere that says what an encoder velocity looks like
	}
}
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
    // it has no relation to wall time. Thus, it is unaffected by leap seconds and
    // NTP updates and the like, but needs a start time to be meaningful..
    private long m_startTime;
	
    // Left motor
    private static final double p_kP = 1, p_kI = 0, p_kD = 0, p_kF = 0;
    //Right motor
    private static final double s_kP = 1, s_kI = 0, s_kD = 0, s_kF = 0;
    
    // FIXME: I have no idea what an encoder velocity is like (See CANTalon.getEncVelocity()), so I don't know what this should be
    private static final double INPUT_RANGE = 4915;
    private static final double ABSOLUTE_TOLERENCE = 4915;
    
    public ReplayMultiPIDCommand(Drivetrain drivetrain) {
    	m_drivetrain = drivetrain;
    	
    	SidePID portPID = new SidePID(m_drivetrain, Drivetrain.MotorSide.Port);
    	SidePID starboardPID = new SidePID(m_drivetrain, Drivetrain.MotorSide.Starboard);
    	
    	m_portPIDController = new PIDController(p_kP, p_kI, p_kD, p_kF, portPID, portPID);
        m_portPIDController.setOutputRange(-1, 1); // Set the output range so that this works with CANTalon.set() in PercentVBus mode
        // FIXME: Once again, I don't know the input range of CANTalon.getEncVelocity
        m_portPIDController.setInputRange(-INPUT_RANGE, INPUT_RANGE); // We limit our input range to revolutions, either direction
        m_portPIDController.setAbsoluteTolerance(ABSOLUTE_TOLERENCE); // This is the tolerance for error for reaching our target, targeting one inch
    	
        m_starboardPIDController = new PIDController(s_kP, s_kI, s_kD, s_kF, starboardPID, starboardPID);
        m_starboardPIDController.setOutputRange(-1, 1); // Set the output range so that this works with CANTalon.set() in PercentVBus mode
        // FIXME: Once again, I don't know the input range of CANTalon.getEncVelocity
        m_starboardPIDController.setInputRange(-INPUT_RANGE, INPUT_RANGE); // We limit our input range to revolutions, either direction
        m_starboardPIDController.setAbsoluteTolerance(ABSOLUTE_TOLERENCE); // This is the tolerance for error for reaching our target, targeting one inch
    	
    	requires(m_drivetrain);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        m_drivetrain.m_logger.info("ReplayMultiPIDCommand initalize");
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
    	m_drivetrain.m_logger.debug("ReplayMultiPID Finished loading, getting, and sorting replay data.");
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        if (m_portPIDController.isEnabled() && m_starboardPIDController.isEnabled()) {
            if (m_curReplayIndex+1 > m_replayData.size()-1) {
                return; // isFinished will return true right after this
            }
            if (System.nanoTime() - m_startTime >= m_replayData.get(m_curReplayIndex+1).timeIndex) {
                m_curReplayIndex++;
                // Catch up if we're more than 1 replay index behind
                while (System.nanoTime() - m_startTime >= m_replayData.get(m_curReplayIndex+1).timeIndex) {
                    if (m_curReplayIndex+1 > m_replayData.size()-1) {
                        return; // isFinished will return true right after this
                    }
                    m_curReplayIndex++;
                }
                if (!m_portPIDController.onTarget()) {
                    m_drivetrain.m_logger.warning("port PID controller is not on target in time.");
                } else if (!m_starboardPIDController.onTarget()) {
                    m_drivetrain.m_logger.warning("starboard PID controller is not on target in time.");
                }
                m_portPIDController.setSetpoint(m_replayData.get(m_curReplayIndex).portVelocity);
                m_starboardPIDController.setSetpoint(m_replayData.get(m_curReplayIndex).starboardVelocity);
            }
        } else {
            m_curReplayIndex = 0;
            m_portPIDController.setSetpoint(m_replayData.get(m_curReplayIndex).portVelocity);
            m_starboardPIDController.setSetpoint(m_replayData.get(m_curReplayIndex).starboardVelocity);
            m_portPIDController.enable();
            m_starboardPIDController.enable();
            m_startTime = System.nanoTime();
        }
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
        m_drivetrain.m_logger.info("ReplayMultiPIDCommand end");
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
        m_drivetrain.m_logger.info("ReplayMultiPIDCommand interrupted");
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
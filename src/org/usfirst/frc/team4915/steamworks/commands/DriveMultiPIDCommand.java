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
public class DriveMultiPIDCommand extends Command {
	
    private final Drivetrain m_drivetrain;
    private PIDController m_portPIDController;
    private PIDController m_starboardPIDController;
    private ArrayList<ReplayDataPoint> m_replayData;
	
    // Left motor
    private static final double p_P = 0, p_I = 0, p_D = 0, p_F = 0;
    //Right motor
    private static final double s_P = 0, s_I = 0, s_D = 0, s_F = 0;
    
    private static final double INPUT_RANGE_MULTIPLIER = 1.3;
    private static final double ABSOLUTE_TOLERENCE = 4915; // FIXME: I have no idea what an encoder velocity is (See CANTalon.getEncVelocity()), so I don't know what this should be
    
    public DriveMultiPIDCommand(Drivetrain drivetrain, ArrayList<ReplayDataPoint> replayData) {
    	m_drivetrain = drivetrain;
    	m_replayData = replayData;
    	
    	SidePID portPID = new SidePID(m_drivetrain, Drivetrain.MotorSide.Port);
    	SidePID starboardPID = new SidePID(m_drivetrain, Drivetrain.MotorSide.Starboard);
    	
        // Get the port upper and lower bounds of ticks by sorting low to high
    	Collections.sort(m_replayData, new Comparator<ReplayDataPoint>() {
             @Override
             public int compare(ReplayDataPoint lhs, ReplayDataPoint rhs) {
                 return lhs.portVelocity > rhs.portVelocity ? -1 : (lhs.portVelocity < rhs.portVelocity ) ? 1 : 0;
             }
        });
    	
    	// Get maximum and minimum ticks on the port side
    	int pMaxTicks = m_replayData.get(m_replayData.size()-1).portVelocity;
    	int pMinTicks = m_replayData.get(0).portVelocity;

        // Get the starboard upper and lower bounds of ticks by sorting low-to-high
    	Collections.sort(m_replayData, new Comparator<ReplayDataPoint>() {
             @Override
             public int compare(ReplayDataPoint lhs, ReplayDataPoint rhs) {
                 return lhs.starboardVelocity > rhs.starboardVelocity ? -1 : (lhs.starboardVelocity < rhs.starboardVelocity ) ? 1 : 0;
             }
        });
    	
    	// Get maximum and minimum ticks on the starboard side
       	int sMaxTicks = m_replayData.get(m_replayData.size()-1).starboardVelocity;
    	int sMinTicks = m_replayData.get(0).starboardVelocity;
    	
    	m_portPIDController = new PIDController(p_P, p_I, p_D, p_F, portPID, portPID);
        m_portPIDController.setOutputRange(-1, 1); // Set the output range so that this works with CANTalon.set() in PercentVBus mode
        m_portPIDController.setInputRange(pMinTicks * INPUT_RANGE_MULTIPLIER, pMaxTicks * INPUT_RANGE_MULTIPLIER); // We limit our input range to revolutions, either direction
        m_portPIDController.setAbsoluteTolerance(ABSOLUTE_TOLERENCE); // This is the tolerance for error for reaching our target, targeting one inch
    	
        m_starboardPIDController = new PIDController(s_P, s_I, s_D, s_F, starboardPID, starboardPID);
        m_starboardPIDController.setOutputRange(-1, 1); // Set the output range so that this works with CANTalon.set() in PercentVBus mode
        m_starboardPIDController.setInputRange(sMinTicks * INPUT_RANGE_MULTIPLIER, sMaxTicks * INPUT_RANGE_MULTIPLIER); // We limit our input range to revolutions, either direction
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
    	
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
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
        m_drivetrain.m_logger.info("DriveStraightCommand end");
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
        m_drivetrain.m_logger.info("DriveStraightCommand interrupted");
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



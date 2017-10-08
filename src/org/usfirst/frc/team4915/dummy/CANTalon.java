package org.usfirst.frc.team4915.dummy;

public class CANTalon {
	
	public static enum TalonControlMode {
		Current,
		Disabled,
		Follower,
		MotionMagic,
		MotionProfile,
		PercentVbus,
		Position,
		Speed,
		Voltage
	}
	
	public static enum FeedbackDevice {
		AnalogEncoder,
		AnalogPot,
		CtreMagEncoder_Absolute,
		CtreMagEncoder_Relative,
		EncFalling,
		EncRising,
		PuseWidth,
		QuadEncoder
	}
	
	private TalonControlMode m_controlMode = TalonControlMode.PercentVbus;
	
	public CANTalon(int dummy) {}
	
	public void changeControlMode(TalonControlMode m) {
		m_controlMode = m;
	}
	
	public void reverseOutput(boolean d) {}
	
	public void setFeedbackDevice(FeedbackDevice d) {}
	
	public void configEncoderCodesPerRev(int d) {}
	
	public void setInverted(boolean d) {}
	
	public void configPeakOutputVoltage(double d0, double d1) {}
	
	public void setVoltageRampRate(double d) {}
	
	public void enableBrakeMode(boolean d) {}
	
	public void set(double d) {}
	
	public void setEncPosition(double d) {}
	
	public TalonControlMode getControlMode() {
		return m_controlMode;
	}
	
	public void setPID(double p, double i, double d, double f, int izone, double closeLoopRampRate, int profile) {}
	
	public void setPosition(double d) {}
	
	public void setProfile(int d) {}
	
	public double get() {
		return 0.0;
	}
	
	public int getEncPosition() {
		return 0;
	}
	
	public double getPosition() {
		return 0.0;
	}
	
	public int getDeviceID() {
		return 0;
	}
	
	public double getOutputCurrent() {
		return 0.0;
	}
	
	public void setAllowableClosedLoopErr(int d) {}
	
	public void reverseSensor(boolean d) {}
	
	public void configNominalOutputVoltage(double d0, double d1) {}
	
	public void setCloseLoopRampRate(double d) {}
	
	public double getEncVelocity() {
		return 0.0;
	}
	
	public double getPulseWidthPosition() {
		return 0.0;
	}
	
	public double getSpeed() {
		return 0.0;
	}
	
	public void setP(double d) {}
	
	public void setI(double d) {}
	
	public void setD(double d) {}
	
	public void setF(double d) {}
}

package org.usfirst.frc.team4915.steamworks;

public class ReplayDataPoint {

	public final int portVelocity;
	public final int starboardVelocity;
	public final long timeIndex;
	
	// ReplayDataPoint contains the cumulative number of ticks, and the monotonic time index at which the data was sampled
	public ReplayDataPoint(int portVelocity, int starboardVelocity, long timeIndex) {
		this.portVelocity = portVelocity;
		this.starboardVelocity = starboardVelocity;
		this.timeIndex = timeIndex;
	}
	
}

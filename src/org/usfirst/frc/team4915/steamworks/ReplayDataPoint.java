package org.usfirst.frc.team4915.steamworks;

public class ReplayDataPoint {

	public final int portTicks;
	public final int starboardTicks;
	public final long time;
	
	// ReplayDataPoint contains the cumulative number of ticks, and the monotonic time index at which the data was sampled
	public ReplayDataPoint(double portTicks, double starboardTicks, long time) {
		this.portTicks = portTicks;
		this.starboardTicks = starboardTicks;
		this.time = time;
	}
	
}

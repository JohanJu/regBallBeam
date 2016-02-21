package oliver;
// PID class to be written by you
public class PID {
	// Current PID parameters
	private PIDParameters p;
	private double I, D; // Integrator state
	private double v; // Desired control signal
	private double e; // Current control error
	private double ad, bd;
	private double yOld, newY;
	
	
	// Constructor
	public PID(String name){
		PIDParameters p = new PIDParameters();
		  p.Beta = 1.0;
		  p.H = 0.1;
		  p.integratorOn = false;
		  p.K = 1;
		  p.Ti = 0.0;
		  p.Tr = 10.0;
		  p.N = 5;
		  new PIDGUI(this, p, name);
		  setParameters(p);
		  ad = p.Td / (p.Td + p.N*p.H);
		  bd = p.K*ad*p.N;
		  this.yOld = 0;
		  this.newY = 0;
		  this.I = 0.0;
		  this.v = 0.0;
		  this.e = 0.0;
	}
	
	// Calculates the control signal v.
	// Called from BallAndBeamRegul.
	public synchronized double calculateOutput(double y, double yref){
		newY = y;
		e = yref - y;
		D = ad*D - bd*(y - yOld);
		v = p.K*(p.Beta*yref - y) + I + D;
		return v;
	}
	
	// Updates the controller state.
	// Should use tracking-based anti-windup
	// Called from BallAndBeamRegul.
	public synchronized void updateState(double u){
		  if (p.integratorOn) {
			  I = I + (p.K * p.H / p.Ti) * e + (p.H / p.Tr) * (u - v);
			  yOld = newY;
		  } else {
			  I = 0.0;
		  }
		
		
		
	}
	
	// Returns the sampling interval expressed as a long.
	// Explicit type casting needed.
	public synchronized long getHMillis(){
		return (long)(p.H*1000);
	}
	
	// Sets the PIDParameters.
	// Called from PIDGUI.
	// Must clone newParameters.
	public synchronized void setParameters(PIDParameters newParameters){
		p = (PIDParameters)newParameters.clone();
		if (!p.integratorOn) {
			I = 0.0;
		}
	  
	}
}
// PID class to be written by you
public class PID {
	// Current PID parameters
	private PIDParameters p;
	private double I;
	private double D;
	private double v;
	private double e;

	// Constructor
	public PID(String name) {
		PIDParameters p = new PIDParameters();
		p.Beta = 1.0;
		p.H = 0.1;
		p.integratorOn = false;
		p.K = 1.0;
		p.Ti = 0.0;
		p.Tr = 10.0;
		new PIDGUI(this, p, name);
		setParameters(p);

		this.I = 0.0;
		this.v = 0.0;
		this.e = 0.0;
	}

	// Calculates the control signal v.
	// Called from BallAndBeamRegul.
	public synchronized double calculateOutput(double y, double yref) {
		e = yref-y;
		v = p.K * p.Beta * e + I + D;
		return v;
	}

	// Updates the controller state.
	// Should use tracking-based anti-windup
	// Called from BallAndBeamRegul.
	public synchronized void updateState(double u) {
		if (p.integratorOn) {
			I += e * p.K * p.H / p.Ti + (u - v) * p.H / p.Tr;
		} else {
			I = 0.0;
		}
	}

	// Returns the sampling interval expressed as a long.
	// Explicit type casting needed.
	public synchronized long getHMillis() {
		return (long) (p.H * 1000.0);
	}

	// Sets the PIDParameters.
	// Called from PIDGUI.
	// Must clone newParameters.
	public synchronized void setParameters(PIDParameters newParameters) {
		p = (PIDParameters) newParameters.clone();
		if (!p.integratorOn) {
			I = 0.0;
		}
	}
}

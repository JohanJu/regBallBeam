package johan;
import SimEnvironment.*;

public class BallAndBeamRegul extends Thread {
	ReferenceGenerator refgen;
	private PID out;
	private PI in;
	private AnalogSource analogInAngle;
	private AnalogSource analogInPosition;
	private AnalogSink analogOut;
	private AnalogSink analogRef;
	private double umin = -10.0;
	private double umax = 10.0;

	public BallAndBeamRegul(ReferenceGenerator refgen, BallAndBeam bb, int pri) {
		this.refgen = refgen;
		analogInPosition = bb.getSource(0);
		analogInAngle = bb.getSource(1);
		analogOut = bb.getSink(0);
		analogRef = bb.getSink(1);
		out = new PID("PID");
		in = new PI("PI");
		setPriority(pri);
	}

	private double limit(double u) {
		if (u < umin) {
			u = umin;
		} else if (u > umax) {
			u = umax;
		}
		return u;
	}

	public void run() {
		long t = System.currentTimeMillis();
		while (true) {
			// outer
			double ref = refgen.getRef();
			double y = analogInPosition.get();
			synchronized (out) {
				double u = limit(out.calculateOutput(y, ref));
				analogOut.set(u);
				out.updateState(u);
				ref = u;
			}
			// inner
			y = analogInAngle.get();
			synchronized (in) {
				double u = limit(in.calculateOutput(y, ref));
				analogOut.set(u);
				in.updateState(u);
			}
			
			
			analogRef.set(ref); // Only for the plotter animation
			t = t + in.getHMillis();
			long duration = t - System.currentTimeMillis();
			if (duration > 0) {
				try {
					sleep(duration);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
			}
		}
	}
}

package oliver;
import SimEnvironment.*;

public class BallAndBeamRegul extends Thread {
	ReferenceGenerator referenceGenerator;
	private PID out;
	private PI in;
	private AnalogSource analogInAngle;
	private AnalogSource analogInPosition;
	private AnalogSink analogOut;
	private AnalogSink analogRef;
	private double umin = -10.0;
	private double umax = 10.0;

	public BallAndBeamRegul(ReferenceGenerator refgen, BallAndBeam bb, int pri) {
		
		analogInPosition = bb.getSource(0);
		analogInAngle = bb.getSource(1);
		analogOut = bb.getSink(0);
		analogRef = bb.getSink(1);
		referenceGenerator = refgen;
		out = new PID("PID");
		in = new PI("PI");
		setPriority(pri);
	}
	private double mm = 10.0;
	private double limit(double u) {
		if (u < -mm) {
			u = -mm;
		} else if (u > mm) {
			u = mm;
		}
		return u;
	}
	private double limit2(double u) {
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
			double ref = referenceGenerator.getRef();
			double y = analogInPosition.get();
			double uref = ref;
			
			synchronized (out) {
				double u = limit(out.calculateOutput(y, ref));
				out.updateState(u);
				uref = u;
			}
			
			// inner
			y = analogInAngle.get();
			
			synchronized (in) {
				double u = limit2(in.calculateOutput(y, uref));
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

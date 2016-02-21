import SimEnvironment.*;

public class BallAndBeamRegul extends Thread {
	ReferenceGenerator refgen;
	private PID cont;
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
		cont = new PID("PID");
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
			// Read inputs
			double y = analogInAngle.get();
			double ref = refgen.getRef();

			synchronized (cont) { // To avoid parameter changes in between
				// Compute control signal
				double u = limit(cont.calculateOutput(y, ref));
//				System.out.println(ref + "\t" + y + "\t" + u);
				analogOut.set(u);
				cont.updateState(u);
			}
			analogRef.set(ref); // Only for the plotter animation

			t = t + cont.getHMillis();
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

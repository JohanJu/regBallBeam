package oliver;
import SimEnvironment.*;

// BallAndBeamRegul class to be written by you
public class BallAndBeamRegul extends Thread {
	// Constructor
	  private AnalogSource analogInAngle;
	  private AnalogSource analogInPosition;
	  private AnalogSink analogOut;
	  private AnalogSink analogRef;
	private ReferenceGenerator referenceGenerator;
	private PID controller;

	
	
	//Define min and max control output
	private double uMin = -10.0;
	private double uMax = 10.0;
	
	public BallAndBeamRegul(ReferenceGenerator refgen, BallAndBeam bb, int priority){
		  // In Constructor
		  analogInPosition = bb.getSource(0);
		  analogInAngle = bb.getSource(1);
		  analogOut = bb.getSink(0);
		  analogRef = bb.getSink(1);
		  referenceGenerator = refgen;
		  controller = new PID("PID");
		  setPriority(priority);
		
	}
	
	public void run(){
		long t = System.currentTimeMillis();
		while (true) {
			// Read inputs
			double y = analogRef.get();
			double ref = referenceGenerator.getRef();
			
			synchronized (controller) { // To avoid parameter changes in between
				// Compute control signal
				double u = limit(controller.calculateOutput(y, ref), uMin, uMax);
				
				// Set output
				analogOut.set(u);
				analogInPosition.set(u);
				analogInAngle.set(u);
				
				// Update state
				controller.updateState(u);
			}
			analogRef.set(ref); // Only for the plotter animation
			
			t = t + controller.getHMillis();
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

		//Saturate output at limits
	private double limit(double u, double umin, double umax) {
		if (u < umin) {
			u = umin;
		} else if (u > umax) {
			u = umax;
		} 
		return u;
	}
	

	
}
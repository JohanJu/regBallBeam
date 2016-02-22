package magnus;
import SimEnvironment.*;
// BallAndBeamRegul class to be written by you
public class BallAndBeamRegul extends Thread {
	  private AnalogSource analogInAngle;
	  private AnalogSource analogInPosition;
	  private AnalogSink analogOut;
	  private AnalogSink analogRef;
	  
	private ReferenceGenerator rg;
	private PID controller;
	private PI cont;
		
	private double uMin=-10.0;
	private double uMax=10.0;
	// Constructor
	public BallAndBeamRegul(ReferenceGenerator refgen, BallAndBeam bb, int priority){
		this.rg=refgen;  
		analogInPosition = bb.getSource(0);
		  analogInAngle = bb.getSource(1);
		  analogOut = bb.getSink(0);
		  analogRef = bb.getSink(1);
		  controller=new PID("PID");
		  cont=new PI("PI");
		  setPriority(priority);
	}
	private double saturation(double u){
		if(u<uMin){
			u=uMin;
		}else if(u>uMax){
			u=uMax;
		}
		return u;
	}

	public void run(){
		long t=System.currentTimeMillis();
		while(true){
			
			double yref=rg.getRef();
			double pos=analogInPosition.get();
			double uref = yref;
			synchronized(controller){
				double u=saturation(controller.calculateOutput(pos, yref));
				System.out.println(yref + "\t" + pos + "\t" + u);
				
				controller.updateState(u);
				uref=u;
			}
			double y=analogInAngle.get();
			synchronized(cont){
				double u=saturation(cont.calculateOutput(y, uref));
				System.out.println(uref + "\t" + y + "\t" + u);
				//set output
				analogOut.set(u);
				//update state
				cont.updateState(u);
			}
			analogRef.set(yref);
			t=t+cont.getHMillis();		//+controller.getHMillis()
			long duration=t-System.currentTimeMillis();
			if(duration>0){
				try{
					sleep(duration);	
				}catch(InterruptedException e){
					e.printStackTrace();
				}
				
			}
			
		}
	}
}

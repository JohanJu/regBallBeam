package magnus;

public class PID {
	private PIDParameters p;
	private double e;
	private double v;
	private double I;
	private double D;
	private double ad;
	private double bd;
	private double yold,newY;
	
	public PID(String name) {
		// TODO Auto-generated constructor stub
		p=new PIDParameters();
		p.Beta=1.0;
		p.H=0.05;	//(0.02-0.1)
		p.integratorOn=false;
		p.Ti=0.0;
		p.Td=4.0;		//(0.5-4)
		p.Tr=10.0;
		p.K=-0.01;	//(-0.2-(-0.01))
		p.N=10.0;		//(5-10)
		
		 
		
		new PIDGUI(this,p,name);
		setParameters(p);
		
		ad=p.Td/(p.H*p.N+p.Td);
		bd=p.K*p.N*ad;
		this.yold = 0;
		  this.newY = 0;
		this.e=0.0;
	
		this.v=0.0;
		this.I=0.0;
		this.D=0.0;	
	}
	
	// Calculates the control signal v.
	// Called from BallAndBeamRegul.
	public synchronized double calculateOutput(double y, double yref){
		e = yref-y;
		D = p.Td*(y-yold);
		double max = p.N;
		if(D>max){
			D = max;
		}else if(D<-max){
			D = -max;
		}
		yold = y;
		v = p.K * p.Beta * e + I + D;
//		System.out.println(D+"\t"+p.K * p.Beta * e+"\t"+v);
		return v;
	}
	
	// Updates the controller state.
	// Should use tracking-based anti-windup
	// Called from BallAndBeamRegul.
	public synchronized void updateState(double u){
		if(p.integratorOn){
			I=I+(p.K*p.H/p.Ti)*e+(p.H/p.Tr)*(u-v);
			yold=newY;
		}else{
			I=0.0;
		}
		
	}
	
	// Returns the sampling interval expressed as a long.
	// Explicit type casting needed.
	public synchronized long getHMillis(){
		return (long) (p.H*1000.0);
	}
	
	// Sets the PIDParameters.
	// Called from PIDGUI.
	// Must clone newParameters.
	public synchronized void setParameters(PIDParameters newParameters){
		p=(PIDParameters)newParameters.clone();
    	if(!p.integratorOn){
    		I=0.0;
    	}
	}

}

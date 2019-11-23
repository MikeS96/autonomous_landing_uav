#ifndef PID_H
#define PID_H

class PID
{


    private:
	double max;
	double min;
	double kp;
	double kd;
	double ki;
	double pre_error;
	double integral;
	double pre_integral;
	
    public:
        // Kp -  proportional gain
        // Ki -  Integral gain
        // Kd -  derivative gain
        // dt -  loop interval time
        // max - maximum value of manipulated variable
        // min - minimum value of manipulated variable
        PID(double cmax, double cmin, double ckp, double ckd, double cki);

        // Returns the manipulated variable given a setpoint and current process value
        double calculate( double setpoint, double pv, double cdt);
        ~PID();

    
};

#endif

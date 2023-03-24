

public class PID {
    private double P, I, D, max_i, min_i, integral, last_error;
    private boolean first_run;


    public PID(double p, double i, double d, int max, int min) {
        this.P = p;
        this.I = i;
        this.D = d;
        this.integral = 0;
        this.max_i = max;
        this.min_i = min;
        this.first_run = true;
    }

    public double update(double error, double dt) {
        if (first_run) {
            last_error = error;
            first_run = false;
        }
        this.integral += I * error * dt;
        double diff = (error - this.last_error) / dt;
        double const_integral = constrain(this.integral, this.max_i);
        double control_out = this.P * error + D * diff + const_integral;
        this.last_error = error;
        return control_out;
    }

    private double constrain(double integral, double max_i) {
        if(integral <= max_i){
            return integral;
        }
        else {
            return max_i;
        }
    }
}
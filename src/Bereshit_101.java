//package Drone;
//import Moon;


import java.text.DecimalFormat;

/**
 * This class represents the basic flight controller of the Bereshit space craft.
 * @author ben-moshe
 *
 */
public class Bereshit_101 {
    public static final double WEIGHT_EMP = 165; // kg
    public static final double WEIGHT_FULE = 420; // kg
    public static final double WEIGHT_FULL = WEIGHT_EMP + WEIGHT_FULE; // kg
    // https://davidson.weizmann.ac.il/online/askexpert/%D7%90%D7%99%D7%9A-%D7%9E%D7%98%D7%99%D7%A1%D7%99%D7%9D-%D7%97%D7%9C%D7%9C%D7%99%D7%AA-%D7%9C%D7%99%D7%A8%D7%97
    public static final double MAIN_ENG_F = 430; // N
    public static final double SECOND_ENG_F = 25; // N
    public static final double MAIN_BURN = 0.15; //liter per sec, 12 liter per m'
    public static final double SECOND_BURN = 0.009; //liter per sec 0.6 liter per m'
    public static final double ALL_BURN = MAIN_BURN + 8*SECOND_BURN;
    double vs = 24.8;
    double hs = 932;
    double dist = 181*1000;
    double ang = 58.3; // zero is vertical (as in landing)
    double alt = 13748; // 2:25:40 (as in the simulation) // https://www.youtube.com/watch?v=JJ0VfRL9AMs
    double time = 0;
    double dt = 1; // sec
    double acc=0; // Acceleration rate (m/s^2)
    double fuel = 121; //
    double weight = WEIGHT_EMP + fuel;
    double NN = 0.7; // rate[0,1]
    boolean start_landing = false;
    boolean landed = false;
    PID pid = null;
    public static double accMax(double weight) {
        return acc(weight, true,8);
    }
    public static double acc(double weight, boolean main, int seconds) {
        double t = 0;
        if(main) {t += MAIN_ENG_F;}
        t += seconds*SECOND_ENG_F;
        double ans = t/weight;
        return ans;
    }
    // 14095, 955.5, 24.8, 2.0

    public void start(){
        this.pid = new PID(0.04,0.003,0.2,1,0);
        if(this.alt<2000){
            this.start_landing = true;
        }
        if(!this.landed){
            this.simulation();
            this.computations();
            this.printData();
        }
    }

    private double get_dvs(){
        if(this.alt> 2000){
            return 30;
        }
        else if(alt > 1000){
            return 24;
        }
        else if(alt > 500){
            return 12;
        }
        else if (alt > 250){
            return 6;
        }
        else if(alt > 125){
            return 2;
        }
        else return 1;
    }

    private void simulation(){
        if(alt>=1){
            if(!start_landing) {	// maintain a vertical speed of [20-25] m/s
                if(vs >25) {NN+=0.003*dt;} // more power for braking
                if(vs <20) {NN-=0.003*dt;} // less power for braking
                if(alt > 3500 && alt < 6000){
                    ang = 57.0;
                }
                else if (alt<3500){
                    ang = 54.0;
                }
            }
            // lower than 2 km - horizontal speed should be close to zero
            else {
                if(this.ang>3) {this.ang-=3.5;} // rotate to vertical position.
                else {this.ang = 0;}
                NN=getNN(); // brake slowly, a proper PID controller here is needed!
                if(hs<2) {hs=0.0;}
                if(alt<125) { // very close to the ground!
                    NN=1; // maximum braking!
                    if(vs<5) {NN=0.0;} // if it is slow enough - go easy on the brakes
                }
                if(alt<5) { // no need to stop
                    NN=0.3;
                }
            }

        }
        else {
            this.landed = true;
        }

    }

    private double getNN() {
        double update = this.pid.update(this.vs - this.get_dvs(),this.dt);
        double ans = Math.max(Math.min(update+NN,1),0);
        return ans;
    }

    private void computations(){
        double ang_rad = Math.toRadians(ang);
        double h_acc = Math.sin(ang_rad)*acc;
        double v_acc = Math.cos(ang_rad)*acc;
        double vacc = Moon.getAcc(hs);
        time+=dt;
        double dw = dt*ALL_BURN*NN;
        if(fuel>0) {
            fuel -= dw;
            weight = WEIGHT_EMP + fuel;
            acc = NN* accMax(weight);
        }
        else { // ran out of fuel
            acc=0;
        }

        v_acc -= vacc;
        if(hs>0) {hs -= h_acc*dt;}
        dist -= hs*dt;
        vs -= v_acc*dt;
        if(vs<0){
            vs = 0;
        }
        alt -= dt*vs;
    }

    private void printData() {
        DecimalFormat form = new DecimalFormat("###.###");
        if(alt<1){
            alt = 0;
        }
        if(!landed && (time % 10 == 0 || alt<100) )
            System.out.println("Time: "+form.format(time)+"  Height: "+form.format(alt)+" Verticalspeed: "
                    +form.format(vs)+" HorizontalSpeed: "+form.format(hs)+ "  fuel:"+form.format(fuel)+"  Weight: "
                    +form.format(weight) +"  Acceleration: " +form.format(acc) +"  Rotation: "
                    +form.format(ang)+ " NN: "+NN);
        if(landed)
            System.out.println("Landed successfully");
    }

    public static void main(String[] args) {
        System.out.println("Simulating Bereshit's Landing:");
        Bereshit_101 bereshit = new Bereshit_101();
        while (!bereshit.landed){
            bereshit.start();
        }
    }
}

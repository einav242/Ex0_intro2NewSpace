//package Drone;
//import Moon;


import java.text.DecimalFormat;
import java.util.Random;

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
    double vs;
    double hs;
    double dist;
    double ang; // zero is vertical (as in landing)
    double alt; // 2:25:40 (as in the simulation) // https://www.youtube.com/watch?v=JJ0VfRL9AMs
    double time;
    double dt; // sec
    double acc; // Acceleration rate (m/s^2)
    double fuel;
    double weight;
    double NN;
    boolean start_landing;
    boolean landed;
    PID pid;
    double update;

    public Bereshit_101(){
        Random rand = new Random();
        double randomNumber = rand.nextDouble();
        this.vs = 20 + (randomNumber * 10);// set the vertical speed to a random number between 20 and 30
        this.hs = 932;
        this.dist = 181*1000;
        this.ang = 58.3; // zero is vertical (as in landing)
        this.alt = 13748; // 2:25:40 (as in the simulation) // https://www.youtube.com/watch?v=JJ0VfRL9AMs
        this.time = 0;
        this.dt = 1; // sec
        this.acc=0; // Acceleration rate (m/s^2)
        this.fuel = 121; //
        this.weight = WEIGHT_EMP + fuel;
        this.NN = 0.7; // rate[0,1]
        this.start_landing = false; //The spacecraft starts landing from a speed of 2000
        this.landed = false; //PID control
        this.pid= new PID(0.04,0.003,0.2,1,0);
    }

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

    public void start(){
        if(this.alt<2000){
            this.start_landing = true;
        }
        if(!this.landed){
            this.simulation();
            this.updateSpeed();
            this.printData();
        }
    }

    // This function returns the desired vertical speed of the spacecraft at each stage of the landing
    private double get_dvs(){
        if(this.alt> 2000){
            return 23;
        }
        else if(alt > 500){
            return 24;
        }
        else if(alt > 200){
            return 12;
        }
        else if (alt > 50){
            return 6;
        }
        else if(alt > 20){
            return 2;
        }
        else return 1;
    }

    // This function sets the angle and necessary power according to the vs and altitude of the spacecraft
    private void simulation(){
        if(alt>=1){
            if(!start_landing) {
                if(vs >25 || vs <20) {
                    NN = getNN();
                }
                if(alt > 3500 && alt < 6000){
                    ang = 60.0;
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
            }

        }
        else {
            this.landed = true;
        }
    }

    // Using PID control, this function returns the necessary force to stop the spacecraft
    private double getNN() {
        this.update = this.pid.update(this.vs - this.get_dvs(),this.dt);
        double ans = Math.max(Math.min(update+NN,1),0);
        return ans;
    }

    // This function updates the vs of the spacecraft and the remaining fuel using acceleration
    private void updateSpeed(){
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
        if(alt<1){
            alt = 0;
        }
        if(!landed && (time % 10 == 0 || alt<100) )
            System.out.println(time+"\t"+vs+"\t"+hs+"\t"+dist+"\t"+alt+"\t"+ang+"\t"+fuel+"\t"+acc+"\t"+get_dvs()
            +"\t"+this.update);
        if(landed)
            System.out.println("Landed successfully");
    }

    public static void main(String[] args) {
        System.out.println("Simulating Bereshit's Landing:");
        System.out.println("time\tvs\ths\tdist\talt\tang\tfuel\tacc\tdvs\tpid");
        Bereshit_101 bereshit = new Bereshit_101();
        while (!bereshit.landed){
            bereshit.start();
        }
    }
}

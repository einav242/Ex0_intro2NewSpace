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
    double vs;
    double hs;
    double dist;
    double ang; // zero is vertical (as in landing)
    double alt; // 2:25:40 (as in the simulation) // https://www.youtube.com/watch?v=JJ0VfRL9AMs
    double time;
    double dt; // sec
    double acc; // Acceleration rate (m/s^2)
    double fuel; //
    double weight;
    double NN; // rate[0,1]
    boolean start_landing;
    boolean landed;
    PID pid;
    Engines engines;

    public Bereshit_101(){
        vs = 24.8;
        hs = 932;
        dist = 181*1000;
        ang = 58.3; // zero is vertical (as in landing)
        alt = 13748; // 2:25:40 (as in the simulation) // https://www.youtube.com/watch?v=JJ0VfRL9AMs
        time = 0;
        dt = 1; // sec
        acc=0; // Acceleration rate (m/s^2)
        fuel = 121; //
        weight = WEIGHT_EMP + fuel;
        NN = 0.7; // rate[0,1]
        start_landing = false;
        landed = false;
        pid = new PID(0.7,1,0.01,1,0);
        engines = new Engines(this);
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


    public void start() {
        if(this.alt < 2000) {
            this.start_landing = true;
        }
        if(!this.landed) {
            this.engines.balance();
            this.NNController();
            double ang_rad = Math.toRadians(ang);
            double h_acc = Math.sin(ang_rad) * acc;
            double v_acc = Math.cos(ang_rad) * acc;
            double vacc = Moon.getAcc(hs);
            time += dt;
            this.AngSpeed();
            this.updateFuel();
            this.update_speed(h_acc, v_acc, vacc);
            this.Data();
        }
    }

    public double constrain(double nn){
        if (nn > 1){
            return 1;
        }
        else if(nn < 0){
            return 0;
        }
        else return nn;
    }

    private void updateAng(){
        if(alt<2000 && this.ang>3) {
            this.engines.balance();
        } // rotate to vertical position.
        else {this.ang =0;}
    }

    private void NNController(){
        if(alt>=1){
            if(alt>2000) {	// maintain a vertical speed of [20-25] m/s
                if(vs >25) {NN= constrain(NN + 0.003*dt);} // more power for braking
                if(vs <20) {NN= constrain(NN - 0.003*dt);} // less power for braking
            }
            // lower than 2 km - horizontal speed should be close to zero
            else {
                double update = this.pid.update(0.5-NN,this.dt);
                NN = constrain(update);
                this.updateAng();
                if(alt<125) { // very close to the ground!
                    NN=1; // maximum braking!
                    if(vs<5) {NN=0.7;} // if it is slow enough - go easy on the brakes
                }
            }
            if(alt<5) { // no need to stop
                NN=0.38;
            }
        }
        else {
            this.landed = true;
        }
    }

    private void update_speed(double h_acc, double v_acc, double vacc){
        if(hs > 0){
            if (hs - h_acc * dt<0){
                hs = 0.1;
            }
            else {
                hs = hs - h_acc * dt;
            }
        }

        if(hs<2 && alt<= 2000){
            hs = 0;
        }

        v_acc -= vacc;
        if(vs - v_acc * dt < 2){
            vs = 0.3;
        }
        else {
            vs = vs - v_acc * dt;
        }
        dist -= hs * dt;
        if (vs < 1) {
            vs = 0;
        }
        alt -= dt * vs;
    }

    private void updateFuel(){
        double dw = dt * ALL_BURN * NN;
        if (fuel > 0) {
            fuel -= dw;
            weight = WEIGHT_EMP + fuel;
            acc = NN * accMax(weight);
        } else { // ran out of fuel
            acc = 0;
        }
    }

    private void AngSpeed(){
        double AngularSpeed = (hs / dist) / Math.toRadians(Math.PI * 2);
        this.ang += AngularSpeed;
        if (ang < 3) {
            ang = 0;
        }
    }


    private void Data() {
        DecimalFormat form = new DecimalFormat("###.###");
        if(alt<0){
            alt = 0;
        }
        if(!landed && (time % 10 == 0 || alt<100) )
            System.out.println("Time: "+form.format(time)+"  Height: "+form.format(alt)+" Verticalspeed: "
                    +form.format(vs)+" HorizontalSpeed: "+form.format(hs)+ "  fuel:"+form.format(fuel)+"  Weight: "
                    +form.format(weight) +"  Acceleration: " +form.format(acc) +"  Rotation: "
                    +form.format(ang));
        if(landed)
            System.out.println("Landed successfully");
    }

    public static void main(String[] args) {
        Bereshit_101 Bereshit_a= new Bereshit_101();
        while(!Bereshit_a.landed) {
            Bereshit_a.start();
        }
    }
}
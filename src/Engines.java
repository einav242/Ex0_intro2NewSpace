
public class Engines {

    public double currentRotation;
    double ang;
    Engine EngineFrontRight1, EngineFrontRight2, EngineFrontLeft1, EngineFrontLeft2, EngineBackRight1,
            EngineBackRight2, EngineBackLeft1, EngineBackLeft2;
    Bereshit_101 Bereshit;

    public Engines(Bereshit_101 bereshit) {
        EngineFrontRight1 = new Engine(this, 1);
        EngineFrontRight2 = new Engine(this,0.75);
        EngineFrontLeft1 = new Engine(this,1);
        EngineFrontLeft2 = new Engine(this,0.75);
        EngineBackRight1 = new Engine(this,-1);
        EngineBackRight2 = new Engine(this,-0.75);
        EngineBackLeft1 = new Engine(this,-1);
        EngineBackLeft2 = new Engine(this,-0.75);
        Bereshit = bereshit;
        this.ang = this.Bereshit.ang;
    }


    public void balance() { //to balance the engines in brishit
        currentRotation = Bereshit.ang;
        if(!this.Bereshit.start_landing) {
            if(currentRotation > ang+4) {
                EngineBackRight1.power();
                EngineBackLeft1.power();
            }
            else if(currentRotation > ang+3) {
                EngineBackRight1.power();
                EngineBackLeft2.power();
            }
            else if(currentRotation > ang+1) {
                EngineBackRight2.power();
            }
            else if(currentRotation < ang+5) {
                EngineFrontRight1.power();
                EngineFrontLeft1.power();
            }
            else if(currentRotation < ang+3) {
                EngineFrontRight1.power();
                EngineFrontLeft2.power();
            }
            else if(currentRotation < ang+1) {
                EngineFrontRight2.power();
            }
        }
        else {
            EngineBackRight1.power();
            EngineFrontRight1.power();
            EngineBackLeft2.power();
        }

    }
    public class Engine {
        double Rotationpower;
        Engines engine;

        public Engine(Engines engines, double powerUp) {
            this.Rotationpower = powerUp;
            engine = engines;
        }

        public void power() {
            if(engine.Bereshit.ang > 0) {
                engine.Bereshit.ang += Rotationpower;
            }
        }
    }



}

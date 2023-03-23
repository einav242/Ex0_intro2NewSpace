
public class PID {
    private double P, I , D, last_error, p0, i0, d0;
    private boolean first_run;
    public PID(double p, double i, double d){
        this.p0 = p;
        this.i0 = i;
        this.d0 = d;
        this.I = 0;
        this.first_run = true;
    }
    public double update(double error, double dt){
        if (first_run){
            last_error = error;
            first_run = false;
        }
        double ans;
        if(error>2000){
            ans = 0.003;
        }
        else{
            this.P = error;
            this.D = (error - this.last_error)/dt;
            this.I+=error;
            this.last_error = error;
            ans = this.P*this.p0 - this.I*this.i0 - this.D*this.d0;
        }
        return ans;
    }

public static void main(String[] args) {
        double targetAltitude, currentAltitude, currentVelocity, currentAcceleration, dt;
        double Kp = 0.1, Ki = 0.01, Kd = 0.5; // ערכי הקבועים לבקרת PID
        double integral = 0, prevError = 0; // ערכי המשתנים לבקרת PID

        System.out.print("Enter the target landing altitude: ");
        targetAltitude = 0;

        currentAltitude = 10000; // נניח שהחללית נמצאת בגובה 10,000 מטרים
        currentVelocity = -500; // נניח שהחללית יורדת במהירות של 500 מטרים לשניה
        currentAcceleration = 0;
        dt = 0.1; // הזמן בין העדכונים לבקרת PID

        while (currentAltitude > targetAltitude) {
        double error = targetAltitude - currentAltitude; // חישוב השגיאה
        integral += error * dt; // חישוב האינטגרל לבקרת PID
        double derivative = (error - prevError) / dt; // חישוב הדרייבטיב לבקרת PID
        double pidOutput = Kp * error + Ki * integral + Kd * derivative; // חישוב הפלט של הבקר PID

        currentAcceleration = pidOutput; // עדכון האצת החללית לפי הפלט של הבקר PID
        currentVelocity += currentAcceleration * dt; // עדכון המהירות של החללית לפי האצתה
        currentAltitude += currentVelocity * dt; // עדכון הגובה של החללית לפי המהירות שלה
        prevError = error; // שמירת השגיאה הקודמת לשימוש בחישוב הדרייבטיב בצעד הבא
        }

        System.out.println("Landing successful!");
        System.out.println("Final altitude: " + currentAltitude);
        System.out.println("Final velocity: " + currentVelocity);
        }
        }
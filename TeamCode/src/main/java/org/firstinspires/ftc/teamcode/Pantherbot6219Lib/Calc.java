package org.firstinspires.ftc.teamcode.Pantherbot6219Lib;

/**
 * Created by KusakabeMirai on 7/22/2018.
 */

public class Calc {

    // This number may seem very trivial, however in a PID Controlled loop system, this is crucial to a critically damped system.
    private static final double EPS = 0.000165;
    private static final double FRICTION = 0.1; // Estimated friction
    private static final double DEADBAND = 0.065; // Corresponding deadband for Logitech F310 Controller

    /**
     * Clear the noises in the analog signal from the controller at the zeroth position
     * @param i input control signal
     * @return adjusted control signal without deadband
     */
    public static double eliminateDeadband(double i) {
        if (Math.abs(i) < DEADBAND) return 0;
        else return (i - DEADBAND * Math.signum(i)) / (1 - DEADBAND);
    }

    /**
     * The clearFriction function handles frictions due to imperfect situations
     * @param speed theoretical output speed
     * @return adjusted output speed
     */
    public static double clearFriction(double speed) {
        if(Math.abs(speed) < EPS) return 0;
        return FRICTION * Math.signum(speed) + speed * (1 - FRICTION);
    }

    /**
     * Desired scaling function for a more responsible output
     * @param i
     * @return
     */
    public static double inputScaling(double i){
        if(i >= 0) return 1.1 * Math.atan(i);
        else return -1.1 * Math.atan(i);
    }
}

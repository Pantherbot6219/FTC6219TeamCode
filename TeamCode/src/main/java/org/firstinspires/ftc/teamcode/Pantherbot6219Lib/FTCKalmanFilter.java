package org.firstinspires.ftc.teamcode.Pantherbot6219Lib;

/**
 * Created by KusakabeMirai on 6/19/2018.
 */

public class FTCKalmanFilter {
    private final String instance;
    private double kQ;
    private double kR;
    private double prevP;
    private double prevXEst;
    private boolean init;

    private static double DEFAULT_KQ = 0.022;
    private static double DEFAULT_KR = 0.617;

    public FTCKalmanFilter(final String instance, double kQ, double kR) {
        this.instance = instance;
        this.kQ = kQ;
        this.kR = kR;
    }

    public FTCKalmanFilter(final String instance) {
        this(instance, DEFAULT_KQ, DEFAULT_KR);
    }

    /**
     * filterData reduces the noise in the data
     *
     * @param data is the data to be filtered by a sensor
     */
    public double filterData(double data) {
        if(!init) {
            prevXEst = data;
            init = true;
        }
        double tempP = prevP + kQ;
        double k = tempP/(tempP + kR);
        double xEst = prevXEst + k * (data - prevXEst);

        prevP = (1 - k) * tempP;
        prevXEst = xEst;

        return prevXEst;

    }

}
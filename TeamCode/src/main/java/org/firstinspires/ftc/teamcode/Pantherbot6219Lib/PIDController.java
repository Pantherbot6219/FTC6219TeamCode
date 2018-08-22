package org.firstinspires.ftc.teamcode.Pantherbot6219Lib;

/**
 * Created by KusakabeMirai on 6/19/2018.
 */
import org.firstinspires.ftc.teamcode.Hallib.HalDashboard;
import org.firstinspires.ftc.teamcode.Hallib.HalUtil;
import org.firstinspires.ftc.teamcode.Robot.TankDriveRobot;

/**
 *
 * The PID Controller is an acronym for "Proportion-Integral-Derivative" Controller, which is a close-loop
 * control mechanism that adjust for changing errors.
 * The main idea follows that the output of a system will be determined on a combination of its error reading
 *              Output = kP * error + kI * error + kD * error + kF * error
 * Where kP, kI, kD, kF are selected constants that will scale the error to the desired output.
 *
 */

/**
 *
 * Advices for tuning a PID Controller:
 1. Set all gains to zero.
 2. Increase the P gain until the response to a disturbance is steady oscillation.
 3. Increase the D gain until the the oscillations go away (i.e. it's critically damped).
 4. Repeat steps 2 and 3 until increasing the D gain does not stop the oscillations.
 5. Set P and D to the last stable values.
 6. Increase the I gain until it brings you to the setpoint with the number of oscillations desired
 (normally zero but a quicker response can be had if you don't mind a couple oscillations of overshoot)
 *
 */

/**
 *  P controls the driving force to overcome the error. The bigger the P, the bigger the response
 *  D controls the rate at which the error is changing. A bigger D means more damp to the error. A critically dampped controll
 *  system is desired.
 *  I controls the additional driving force to overcome an outside influence to the robot, i.e. gravity.
 *
 *  a good visual demonstration can be found in the following link:
 *  https://www.youtube.com/watch?v=4Y7zG48uHRo
 */

public class PIDController {

    private HalDashboard dashboard;
    private double kP;
    private double kI;
    private double kD;
    private double kF;
    private double tolerance;
    private double settlingTime;
    private PidInput pidInput;

    private boolean inverted = false;
    private boolean absSetPoint = false;
    private boolean noOscillation = false;
    private double minTarget = 0.0;
    private double maxTarget = 0.0;
    private double minOutput = -1.0;
    private double maxOutput = 1.0;

    private double prevTime = 0.0;
    private double prevError = 0.0;
    private double totalError = 0.0;
    private double settlingStartTime = 0.0;
    private double setPoint = 0.0;
    private double setPointSign = 1.0;
    private double input = 0.0;
    private double output = 0.0;

    private double pTerm;
    private double iTerm;
    private double dTerm;
    private double fTerm;

    public interface PidInput {
        double getInput(PIDController pidCtrl);
    }

    // Constructor
    public PIDController(double kP, double kI, double kD, double kF, double tolerance, double settlingTime, PidInput pidInput) {
        dashboard = TankDriveRobot.getDashboard(); //same type of dashboard applied for mecanum drive based robot
        this.kP = kP; // The proportion of the error
        this.kI = kI; // The amount of time spent with the error, or the total error, hence "integrated" error
        this.kD = kD; // The speed at which the error is changing
        this.kF = kF; // Predicting the future error and therefore adjust to it
        this.tolerance = tolerance; // How much error do we tolerance for a PID action
        this.settlingTime = settlingTime; // How much time do the PID settle to our target
        this.pidInput = pidInput; // Input value
    }

    public void displayPidInfo(int lineNum) {
        dashboard.displayPrintf(lineNum, "Target=%.1f, Input=%.1f, Error=%.1f", setPoint, pidInput.getInput(this), prevError);
        dashboard.displayPrintf(lineNum + 1, "minOutput=%.1f, Output=%.1f, maxOutput=%.1f", minOutput, output, maxOutput);
        System.out.printf("Target=%.1f, Input=%.1f, Error=%.1f", setPoint, pidInput.getInput(this), prevError);
        System.out.println();
        System.out.printf("minOutput=%.1f, Output=%.1f, maxOutput=%.1f", minOutput, output, maxOutput);
    }

    public void setTargetRange(double minTarget, double maxTarget) {
        this.minTarget = minTarget;
        this.maxTarget = maxTarget;
    }

    //Inverts information if needed
    public void setInverted(boolean inverted) {
        this.inverted = inverted;
    }

    public void setAbsoluteSetPoint(boolean absolute) {
        this.absSetPoint = absolute;
    }

    public void setPID(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }

    //Sets target in ticks
    public void setTarget(double target) {
        double input = pidInput.getInput(this);
        if (!absSetPoint) {
            // Set point is relative, add target to current input to get absolute set point.
            setPoint = input + target;
            prevError = target;
            displayPidInfo(0);
        }
        else {
            // Set point is absolute, use as is.
            setPoint = target;
            prevError = setPoint - input;
            displayPidInfo(0);
        }

        setPointSign = Math.signum(prevError);

        // If there is a valid target range, limit the set point to this range.
        if (maxTarget > minTarget) {
            if (setPoint > maxTarget) setPoint = maxTarget;
            else if (setPoint < minTarget) setPoint = minTarget;

            displayPidInfo(0);
        }

        prevTime = HalUtil.getCurrentTime();

        if (inverted) prevError = -prevError;

        totalError = 0.0;
        settlingStartTime = HalUtil.getCurrentTime();
    }

    //Returns previous error
    public double getError() {
        return prevError;
    }

    // reset method. Should be ran every time a PID action is completed
    public void reset() {
        prevError = 0.0;
        prevTime = 0.0;
        totalError = 0.0;
        setPoint = 0.0;
        setPointSign = 1.0;
        output = 0.0;
    }

    //Determines whether or not the robot is on track
    public boolean isOnTarget() {
        boolean onTarget = false;

        if (noOscillation) {
            // Don't allow oscillation, so if we are within tolerance or we pass target, just quit.
            if (prevError*setPointSign <= tolerance) onTarget = true;
        }

        else if (Math.abs(prevError) > tolerance) settlingStartTime = HalUtil.getCurrentTime();
        else if (HalUtil.getCurrentTime() >= settlingStartTime + settlingTime) onTarget = true;

        return onTarget;
    }

    //setOutputRange
    public void setOutputRange(double minOutput, double maxOutput) {
        this.minOutput = minOutput;
        this.maxOutput = maxOutput;
    }

    //Calculates the power output
    public double getPowerOutput() {
        double currTime = HalUtil.getCurrentTime(); // current time stamp
        double deltaTime = currTime - prevTime;

        prevTime = currTime;
        input = pidInput.getInput(this);

        double error = setPoint - input;

        if (inverted) error = -error;

        if (kI != 0.0) {
            // Making sure the error is bounded by the max/min output range
            double potentialGain = (totalError + error * deltaTime) * kI;
            if (potentialGain >= maxOutput) totalError = maxOutput / kI;
            else if (potentialGain > minOutput) totalError += error * deltaTime;
            else totalError = minOutput / kI;
        }

        pTerm = kP * error;
        iTerm = kI * totalError;
        dTerm = (deltaTime > 0.0 ? kD * (error - prevError) / deltaTime : 0.0);
        fTerm = kF * setPoint;
        output = fTerm + pTerm + iTerm + dTerm; //complete PID Output

        //bound output
        prevError = error;
        if (output > maxOutput) output = maxOutput;
        else if (output < minOutput)output = minOutput;

        return output;
    }

}
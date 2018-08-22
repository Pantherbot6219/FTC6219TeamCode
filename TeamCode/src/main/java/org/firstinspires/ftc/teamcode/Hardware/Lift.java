package org.firstinspires.ftc.teamcode.Hardware;

/**
 * Created by KusakabeMirai on 6/19/2018.
 */
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Hallib.HalDashboard;
import org.firstinspires.ftc.teamcode.Hallib.HalUtil;
import org.firstinspires.ftc.teamcode.Pantherbot6219Lib.PIDController;
import org.firstinspires.ftc.teamcode.Robot.RobotMap;

public class Lift implements PIDController.PidInput{

    public boolean isTargetReached = true;

    private final int LIFT_ZERO = 0;
    private final int LIFT_ONE_CUBE = 1;
    private final int LIFT_TWO_CUBE = 2;
    private final int LIFT_THREE_CUBE = 3;
    private final int LIFT_MAX = 4;

    private final double LIFT_ZERO_POS = 0;
    private final double LIFT_ONE_CUBE_POS = 0.1;
    private final double LIFT_TWO_CUBE_POS = 0.2;
    private final double LIFT_THREE_CUBE_POS = 0.3;
    private final double LIFT_MAX_POS = 0.4;

    private static final double
            FEED=0.05,
            MAX_ACCELERATION_PER_MS=0.0045,
            MAX_ACCELERATION_DISCRETE=0.2,
            MAX_POWER=0.8,
            MAX_POWER_MANUAL_UP=0.9,
            MAX_POWER_MANUAL_DOWN=0.7,
            HEIGHT_PER_ROTATION=0.06,
            GEAR=1,
            SIGNALS_PER_ROTATION=4096,
            REST_BOUND=0.02,
            SLOW_BOUND=0.2,
            SLOW_BOUND_VELOCITY=0.53;

    private static final double MOTOR_DIRECTION=-1;
    private static final boolean INVERTED=true;

    private volatile double mLastSpeed;
    private volatile long mLastTime;

    private int liftPosID = 0;

    private boolean active = false;

    private double prevPos = 0;

    private double stallStartTime;
    private DcMotor lift;
    private LinearOpMode opMode;
    private PIDController pidController;
    /**
     * Lift is a constructor
     *
     * @param opMode is the opMode in the main program
     */
    public Lift(LinearOpMode opMode){
        this.opMode = opMode;
        pidController = new PIDController(0.03,0,0,0,0.8,0.2,this);
        lift = opMode.hardwareMap.dcMotor.get(RobotMap.DcMotor.lift);
        if(INVERTED)lift.setDirection(DcMotor.Direction.REVERSE);
        else lift.setDirection(DcMotor.Direction.FORWARD);
    }

    /**
     * up sets the lift to a certain position
     * @param liftPosition is the desired position
     */

    public void liftController (int liftPosition, boolean isActivate){
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftPosID = liftPosition;
        active = isActivate;
        if(active) {
            switch (liftPosID) {
                case (LIFT_ZERO):
                    gotoPos(LIFT_ZERO_POS);
                    break;
                case (LIFT_ONE_CUBE):
                    gotoPos(LIFT_ONE_CUBE_POS);
                    break;
                case (LIFT_TWO_CUBE):
                    gotoPos(LIFT_TWO_CUBE_POS);
                    break;
                case (LIFT_THREE_CUBE):
                    gotoPos(LIFT_THREE_CUBE_POS);
                    break;
                case (LIFT_MAX):
                    gotoPos(LIFT_MAX_POS);
                    break;
                default:
                    break;
            }
        }
    }

    public void gotoPos(double pos){
        pidController.reset();
        pidController.setTarget(pos);
        stallStartTime = HalUtil.getCurrentTime();
        isTargetReached = false;
        if (!pidController.isOnTarget() && opMode.opModeIsActive()){
            int currLiftPos = lift.getCurrentPosition();
            double liftPower = pidController.getPowerOutput();
            lift.setPower(liftPower);
            double currTime = HalUtil.getCurrentTime();
            if (currLiftPos != prevPos) {
                stallStartTime = currTime;
                prevPos = currLiftPos;
            }
            else if (currTime > stallStartTime + 0.15) {
                // lol that means the lift is locked, good luck fixing that
                active = false;
            }
            pidController.displayPidInfo(5);
        }
        else{
            isTargetReached = true;
            active = false;
            feedStop();

        }
    }

    public void liftglyph(){
        HalUtil.sleep(400);
        lift.setPower(1);
        HalUtil.sleep(300);
        lift.setPower(0);
    }

    public void resetLift(){
        lift.setPower(0);
        //lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void control(double input) {
        input*=(input>0)?MAX_POWER_MANUAL_UP:MAX_POWER_MANUAL_DOWN;
        setSpeed(boundSpeed(boundAcceleration(input)));
    }

    public void setStatic() {
        if(isSensorPluggedIn()) {
            gotoPos(pidGet());
        }else {
            setSpeed(FEED);
        }
    }

    public void feedStop() {
        pidController.reset();
        resetLift();
        mLastSpeed=0;
    }


    public double pidGet() {
        return signalToMeter(lift.getCurrentPosition());
    }

    private double boundSpeed(double speed) {
        if(speed<0) {
            if(pidGet()<=REST_BOUND) {
                return 0;
            }else if(pidGet()<=SLOW_BOUND) {
                return Math.max(speed, -SLOW_BOUND_VELOCITY);
            }
        }else {

        }
        return speed;
    }

    private double boundAcceleration(double speed) {
        double bound = (System.currentTimeMillis() - mLastTime) * MAX_ACCELERATION_PER_MS;
        bound = Math.min(bound, MAX_ACCELERATION_DISCRETE);
        if(Math.abs(speed - mLastSpeed) > bound) {
            speed = mLastSpeed + Math.signum(speed - mLastSpeed) * bound;
        }
        return speed;
    }

    private void setSpeed(double speed) {
        mLastSpeed = speed;
        mLastTime = System.currentTimeMillis();
        if(speed < 0) lift.setPower(speed * MOTOR_DIRECTION); // test up and down, then decide if apply any slow down factor or not
        else lift.setPower(speed * MOTOR_DIRECTION);
    }

    private static double signalToMeter(int count) {
        return count * HEIGHT_PER_ROTATION/(GEAR * SIGNALS_PER_ROTATION);
    }

    private boolean isSensorPluggedIn() {
        return lift.getMode() == DcMotor.RunMode.RUN_USING_ENCODER;
    }

    public double getInput(PIDController pidCtrl) {
        double input = 0.0;
        if (pidCtrl == pidController) {
            input = lift.getCurrentPosition();
        }

        return input;
    }

}
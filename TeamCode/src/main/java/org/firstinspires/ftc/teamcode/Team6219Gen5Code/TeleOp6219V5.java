package org.firstinspires.ftc.teamcode.Team6219Gen5Code;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by KusakabeMirai on 11/2/2017.
 */
@TeleOp(name="Pantherbot6219 : TeleOp6219V5", group="Pantherbot6219")
public class TeleOp6219V5 extends OpMode {
    public Gen5Hardware robot               = null;
    public final double boxHeight           = 6.5;
    public double multiplier                = 0.8;
    public double slowDownRate              = 0.6;
    public double leftFrontPower            = 0;
    public double leftBackPower             = 0;
    public double rightFrontPower           = 0;
    public double rightBackPower            = 0;
    public double liftPower                 = 0;
    public double slidePower                = 0;
    public double intakePower               = 0;
    public double jointPos                  = 0;
    public final boolean ACTIVE             = true;
    public final boolean PASSIVE            = false;
    public boolean endgame                  = false;
    public boolean READY                    = true;
    public boolean READY2                   = true;
    public boolean READY3                   = true;
    public boolean READY4                   = true;
    public boolean READY5                   = true;
    public boolean READY6                   = true;
    public final int RED                    = 1;
    public final int BLUE                   = 0;
    public int counter                      = 0;
    public int counter2                     = 0;
    public int counter3                     = 0;
    public int counter4                     = 0;
    public int counter5                     = 0;
    public int targetHeight                 = 0;

    @Override
    public void start(){
    }

    @Override
    public void init(){
        robot = new Gen5Hardware(hardwareMap,BLUE);
    }

    @Override
    public void loop(){
        if((gamepad2.right_bumper)&&(counter == 0)&&(READY)){
            robot.dump1.setPosition(.85);
            robot.dump2.setPosition(0.95);
            counter = 1;
            READY = false;
        }

        if((gamepad2.right_bumper)&&(counter == 1)&&(READY)){
            robot.dump1.setPosition(0.13);
            robot.dump2.setPosition(0.26);
            counter = 0;
            READY = false;
        }

        if((!gamepad2.right_bumper)&&(!READY)){
            READY = true;
        }

        if((gamepad2.y)){
            liftPower = -0.5;
        }
        else if(gamepad2.a){
            liftPower = .5;
        }
        else{
            liftPower = 0;
        }

        if(Math.abs(gamepad2.right_trigger) > 0 && counter3 == 0 && READY3){
            robot.joint.setPosition(1);
            counter3 = 1;
            READY3 = false;
        }
        else if(gamepad2.right_trigger > 0 && counter3 == 1 && READY3){
            robot.joint.setPosition(0);
            counter3 = 0;
            READY3 = false;
        }
        else if(gamepad2.right_trigger == 0 && (!READY3)){
            READY3 = true;
        }

        if(Math.abs(gamepad2.left_trigger) > 0 && counter4 == 0 && READY4){
            robot.claw.setPosition(1);
            counter4 = 1;
            READY4 = false;
        }
        else if(gamepad2.left_trigger > 0 && counter4 == 1 && READY4){
            robot.claw.setPosition(0);
            counter4 = 0;
            READY4 = false;
        }
        else if(gamepad2.left_trigger == 0 && (!READY4)){
            READY4 = true;
        }

        if((gamepad2.b)){
            slidePower = 0.6;
        }

        else if(gamepad2.x){
            slidePower = -.6;
        }

        else{
            slidePower = 0;
        }

        double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4; //atan2 is an arctan function taht is used for polar cordinates
        double rightX = scaleInput(Range.clip(gamepad1.right_stick_x,-1, 1)) * slowDownRate;
        /*leftFrontPower = r * Math.cos(robotAngle) * multiplier + rightX;
        rightFrontPower = r * Math.sin(robotAngle) * multiplier - rightX;
        leftBackPower = r * Math.sin(robotAngle) * multiplier + rightX;
        rightBackPower = r * Math.cos(robotAngle) * multiplier - rightX;*/
        /*new*/
        rightFrontPower = r * Math.cos(robotAngle) * multiplier + rightX;
        leftFrontPower = r * Math.sin(robotAngle) * multiplier - rightX;
        rightBackPower = r * Math.sin(robotAngle) * multiplier + rightX;
        leftBackPower = r * Math.cos(robotAngle) * multiplier - rightX;
        robot.motorLeftFront.setPower(leftFrontPower);
        robot.motorRightFront.setPower(rightFrontPower);
        robot.motorLeftBack.setPower(leftBackPower);
        robot.motorRightBack.setPower(rightBackPower);
        robot.slide.setPower(slidePower);
        robot.lift.setPower(liftPower);
        robot.leftIntake.setPower(scaleIntake(Range.clip(gamepad2.left_stick_y,-1, 1)));
        robot.rightIntake.setPower(scaleIntake(Range.clip(gamepad2.right_stick_y,-1, 1)));
        telemetry.addData("range1: ", robot.rangeSensor1.getDistance(DistanceUnit.CM));
        telemetry.addData("range2: ", robot.rangeSensor2.getDistance(DistanceUnit.CM));
        telemetry.addData("gyro: ", robot.gyroSensor.getIntegratedZValue());
        telemetry.addData("color number: ", robot.colorSensor.colorNumber());
        telemetry.update();
    }

    private boolean isAtTargetThreshold(int target, int current, double threshold) {
        int error = target - current;
        if (Math.abs(error) < threshold) {
            return true;
        } else {
            return false;
        }
    }

    double scaleInput(double dVal)  {
        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);

        // index should be positive.
        if (index < 0) {
            index = -index;
        }

        // index cannot exceed size of array minus 1.
        if (index > 16) {
            index = 16;
        }

        // get value from the array.
        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        // return scaled value.
        return dScale;
    }

    double scaleIntake(double dVal)  {
        double[] scaleArray = { 0.0, 0.65, 0.69, 0.70, 0.72, 0.75, 0.78, 0.84,
                0.88, 0.91, 0.93, 0.94, 0.95, 0.95, 0.95, 0.95, 0.95 };

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);

        // index should be positive.
        if (index < 0) {
            index = -index;
        }

        // index cannot exceed size of array minus 1.
        if (index > 16) {
            index = 16;
        }

        // get value from the array.
        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        // return scaled value.
        return dScale;
    }



    @Override
    public void stop(){

    }
}

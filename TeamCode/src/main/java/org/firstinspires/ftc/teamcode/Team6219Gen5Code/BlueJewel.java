package org.firstinspires.ftc.teamcode.Team6219Gen5Code;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by KusakabeMirai on 9/22/2017.
 */

@Autonomous (name="Autonomous : BlueJewel", group="PantherbotV5")
public class BlueJewel extends LinearOpMode {
    Gen5Hardware robot                              = null;
    public OpenGLMatrix lastLocation                = null;
    public VuforiaLocalizer vuforia                 = null;
    public String vuMarkVal                         = null;    //translate the VuMarkSample into a String for output
    public int vuMarkValIn                          = -1;
    public boolean ACTIVE                           = true;
    public boolean PASSIVE                          = false;
    public final int RED                            = 1;
    public final int BLUE                           = -1;
    public final int BLACK                          = 0;
    public double colorVal                          = 0;
    public boolean detected                         = false;
    public static double distanceToWall             = -1;
    public static final double HEADING_THRESHOLD    = 1 ;      // As tight as we can make it with an integer gyro
    public static final double P_TURN_COEFF         = 0.05;     // Larger is more responsive, but also less stable
    public static final double P_DRIVE_COEFF        = 0.075;    // Larger is more responsive, but also less stable
    public static final double COUNTS_PER_MOTOR_REV = 1440;   // eg: TETRIX Motor Encoder
    public static final double DRIVE_GEAR_REDUCTION = 3.0;    // This is < 1.0 if geared UP
    public static final double WHEEL_DIAMETER_INCHES= 6.0;    // For figuring circumference
    public static final double COUNTS_PER_INCH      = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415926);
    public ElapsedTime timer                      = new ElapsedTime();
    public double colors[] = new double[3];

    public void setup(){
        this.robot = new Gen5Hardware(hardwareMap,BLUE);
        for(int i = 0; i < colors.length; i++){
            colors[i] = 0;
        }
    }

    public void stop(long time){
        robot.motorRightFront.setPower(0);
        robot.motorLeftFront.setPower(0);
        robot.motorRightBack.setPower(0);
        robot.motorLeftBack.setPower(0);
        robot.lift.setPower(0);
        robot.leftIntake.setPower(0);
        robot.rightIntake.setPower(0);
        sleep(time);
    }

    public void backToEnd(){
        while(robot.rangeSensor1.cmUltrasonic() > 12.5){
            robot.motorRightFront.setPower(-.18);
            robot.motorLeftFront.setPower(-.18);
            robot.motorRightBack.setPower(-.18);
            robot.motorLeftBack.setPower(-.18);
        }
        robot.motorRightFront.setPower(0);
        robot.motorLeftFront.setPower(0);
        robot.motorRightBack.setPower(0);
        robot.motorLeftBack.setPower(0);
    }

    public void lift(long time){
        robot.lift.setPower(-0.5);
        sleep(time);
    }

    public void liftDown(long time){
        robot.lift.setPower(0.5);
        sleep(time);
    }

    public void forward(double power, long time) {
        robot.motorRightFront.setPower(power);
        robot.motorLeftFront.setPower(power);
        robot.motorRightBack.setPower(power);
        robot.motorLeftBack.setPower(power);
        sleep(time);
    }

    public void forward(double power) {
        robot.motorRightFront.setPower(power);
        robot.motorLeftFront.setPower(power);
        robot.motorRightBack.setPower(power);
        robot.motorLeftBack.setPower(power);
    }

    public void fTransis(long time){
        robot.motorRightFront.setPower(0.12);
        robot.motorLeftFront.setPower(0.12);
        robot.motorRightBack.setPower(0.12);
        robot.motorLeftBack.setPower(.12);
        sleep(time);
    }

    public void bTransis(long time){
        robot.motorRightFront.setPower(-.12);
        robot.motorLeftFront.setPower(-.12);
        robot.motorRightBack.setPower(-.12);
        robot.motorLeftBack.setPower(-.12);
        sleep(time);
    }

    public void turnRight(long time){
        robot.motorRightFront.setPower(.6);
        robot.motorLeftFront.setPower(-.6);
        robot.motorRightBack.setPower(.6);
        robot.motorLeftBack.setPower(-.6);
        sleep(time);
    }

    public void turnLeft(long time){
        robot.motorRightFront.setPower(-.6);
        robot.motorLeftFront.setPower(.6);
        robot.motorRightBack.setPower(-.6);
        robot.motorLeftBack.setPower(.6);
        sleep(time);
    }

    public void backward(long time){
        robot.motorRightFront.setPower(-.7);
        robot.motorLeftFront.setPower(-.7);
        robot.motorRightBack.setPower(-.7);
        robot.motorLeftBack.setPower(-.7);
        sleep(time);
    }

    public void backToEndRed(double dist){
        while(getRightDist() > dist){
            robot.motorRightFront.setPower(-.2);
            robot.motorLeftFront.setPower(-.2);
            robot.motorRightBack.setPower(-.2);
            robot.motorLeftBack.setPower(-.2);
        }
        stop(100);
    }

    public void backward(double power, long time){
        robot.motorRightFront.setPower(-power);
        robot.motorLeftFront.setPower(-power);
        robot.motorRightBack.setPower(-power);
        robot.motorLeftBack.setPower(-power);
        sleep(time);
    }

    public void rightward(long time){
        robot.motorRightFront.setPower(-.1);
        robot.motorLeftFront.setPower(.1);
        robot.motorRightBack.setPower(.1);
        robot.motorLeftBack.setPower(.1);
        sleep(time);
    }

    public void leftward(long time){
        robot.motorRightFront.setPower(.3);
        robot.motorLeftFront.setPower(-.3);
        robot.motorRightBack.setPower(-.3);
        robot.motorLeftBack.setPower(.3);
        sleep(time);
    }

    public void dump(){
        robot.dump1.setPosition(0.85);
        robot.dump2.setPosition(0.95);
        stop(200);
        encoderDrive(0.4, 1.5, 1.5, 15);
        robot.dump1.setPosition(0.13);
        robot.dump2.setPosition(0.23);
        stop(200);
    }

    public void correctAngle(){
        int reading = robot.gyroSensor.getHeading();
        if(robot.gyroSensor.getHeading() > 300) reading = reading - 360;
        if(reading < 0){
            while(reading < 0){
                robot.motorRightFront.setPower(-.1);
                robot.motorLeftFront.setPower(.1);
                robot.motorRightBack.setPower(-.1);
                robot.motorLeftBack.setPower(.1);
                if(robot.gyroSensor.getHeading() > 300) reading = robot.gyroSensor.getHeading() - 360;
                else break;
                telemetry.addData("gyro", "X:" + robot.gyroSensor.rawX() / 100 + " Y:" + robot.gyroSensor.rawY() / 100 + " Z:" + robot.gyroSensor.rawZ() / 100);
                telemetry.addData("gyroHead", String.valueOf(robot.gyroSensor.getHeading()));
                telemetry.update();
            }
            robot.motorRightFront.setPower(0);
            robot.motorLeftFront.setPower(0);
            robot.motorRightBack.setPower(0);
            robot.motorLeftBack.setPower(0);
        }
    }

    public void turnRight90Red(){
        while(robot.gyroSensor.isCalibrating()) {

        }
        timer.reset();
        while ((robot.gyroSensor.getIntegratedZValue() > -90) && (timer.seconds()< 4.55) && opModeIsActive()) {
            robot.motorRightFront.setPower(-.4);
            robot.motorLeftFront.setPower(.4);
            robot.motorRightBack.setPower(-.4);
            robot.motorLeftBack.setPower(.4);
            telemetry.addData("gyro", "X:" + robot.gyroSensor.rawX() / 100 + " Y:" + robot.gyroSensor.rawY() / 100 + " Z:" + robot.gyroSensor.rawZ() / 100);
            telemetry.addData("gyro:", String.valueOf(robot.gyroSensor.getIntegratedZValue()));
            telemetry.update();
        }
        telemetry.addData("gyro: ", String.valueOf(robot.gyroSensor.getIntegratedZValue()));
        telemetry.update();
        robot.motorRightFront.setPower(0);
        robot.motorLeftFront.setPower(0);
        robot.motorRightBack.setPower(0);
        robot.motorLeftBack.setPower(0);
        stop(100);
        timer.reset();
        while ((robot.gyroSensor.getIntegratedZValue() < -90)&&(timer.seconds()< 4.55) && opModeIsActive()) {
            robot.motorRightFront.setPower(.06);
            robot.motorLeftFront.setPower(-.06);
            robot.motorRightBack.setPower(.06);
            robot.motorLeftBack.setPower(-.06);
            telemetry.addData("gyro", "X:" + robot.gyroSensor.rawX() / 100 + " Y:" + robot.gyroSensor.rawY() / 100 + " Z:" + robot.gyroSensor.rawZ() / 100);
            telemetry.addData("gyroHead", String.valueOf(robot.gyroSensor.getIntegratedZValue()));
            telemetry.update();
        }
        while ((robot.gyroSensor.getIntegratedZValue() > -90) && (timer.seconds()< 4.55) && opModeIsActive()) {
            robot.motorRightFront.setPower(-.04);
            robot.motorLeftFront.setPower(.04);
            robot.motorRightBack.setPower(-.04);
            robot.motorLeftBack.setPower(.04);
            telemetry.addData("gyro", "X:" + robot.gyroSensor.rawX() / 100 + " Y:" + robot.gyroSensor.rawY() / 100 + " Z:" + robot.gyroSensor.rawZ() / 100);
            telemetry.addData("gyro:", String.valueOf(robot.gyroSensor.getIntegratedZValue()));
            telemetry.update();
        }

    }

    public void hitForward(){
        robot.horizontalArm.setPosition(0.1);
        sleep(300);
        robot.horizontalArm.setPosition(0.37);
        sleep(200);
        robot.horizontalArm.setPosition(0.39);
        sleep(200);
        robot.verticalArm.setPosition(0.5);
        sleep(300);
        robot.horizontalArm.setPosition(0.45);
        sleep(200);
        robot.verticalArm.setPosition(0.7);
        sleep(200);
    }

    public void hitBackward(){
        robot.horizontalArm.setPosition(0.8);
        sleep(300);
        robot.horizontalArm.setPosition(0.6);
        sleep(200);
        robot.horizontalArm.setPosition(0.46);
        sleep(200);
        robot.verticalArm.setPosition(0.5);
        sleep(300);
        robot.horizontalArm.setPosition(0.45);
        sleep(200);
        robot.verticalArm.setPosition(0.7);
        sleep(200);
    }

    public void turnRight90Blue(){
        while(robot.gyroSensor.isCalibrating()) {

        }
        timer.reset();
        while ((robot.gyroSensor.getIntegratedZValue() > -90) && (timer.seconds()< 2.55)) {
            robot.motorRightFront.setPower(.4);
            robot.motorLeftFront.setPower(-.4);
            robot.motorRightBack.setPower(.4);
            robot.motorLeftBack.setPower(-.4);
            telemetry.addData("gyro", "X:" + robot.gyroSensor.rawX() / 100 + " Y:" + robot.gyroSensor.rawY() / 100 + " Z:" + robot.gyroSensor.rawZ() / 100);
            telemetry.addData("gyroHead", robot.gyroSensor.getIntegratedZValue());
            telemetry.update();
        }
        telemetry.addData("gyroHead", String.valueOf(robot.gyroSensor.getHeading()));
        telemetry.update();
        stop(100);
        timer.reset();
        while ((robot.gyroSensor.getIntegratedZValue() < -90) && (timer.seconds()< 4.55)) {
            robot.motorRightFront.setPower(-.06);
            robot.motorLeftFront.setPower(.06);
            robot.motorRightBack.setPower(-.06);
            robot.motorLeftBack.setPower(.06);
            telemetry.addData("gyro", "X:" + robot.gyroSensor.rawX() / 100 + " Y:" + robot.gyroSensor.rawY() / 100 + " Z:" + robot.gyroSensor.rawZ() / 100);
            telemetry.addData("gyroHead", String.valueOf(robot.gyroSensor.getHeading()));
            telemetry.update();
        }
        while ((robot.gyroSensor.getIntegratedZValue() > -90) && (timer.seconds()< 2.55)) {
            robot.motorRightFront.setPower(.04);
            robot.motorLeftFront.setPower(-.04);
            robot.motorRightBack.setPower(.04);
            robot.motorLeftBack.setPower(-.04);
            telemetry.addData("gyro", "X:" + robot.gyroSensor.rawX() / 100 + " Y:" + robot.gyroSensor.rawY() / 100 + " Z:" + robot.gyroSensor.rawZ() / 100);
            telemetry.addData("gyroHead", robot.gyroSensor.getIntegratedZValue());
            telemetry.update();
        }
    }

    public void turn180(){
        while(robot.gyroSensor.isCalibrating()) {

        }
        timer.reset();
        while ((robot.gyroSensor.getIntegratedZValue() < 180) && (timer.seconds()< 3.55)) {
            robot.motorRightFront.setPower(-.7);
            robot.motorLeftFront.setPower(.7);
            robot.motorRightBack.setPower(-.7);
            robot.motorLeftBack.setPower(.7);
            telemetry.addData("gyro", "X:" + robot.gyroSensor.rawX() / 100 + " Y:" + robot.gyroSensor.rawY() / 100 + " Z:" + robot.gyroSensor.rawZ() / 100);
            telemetry.addData("gyro:", String.valueOf(robot.gyroSensor.getIntegratedZValue()));
            telemetry.update();
        }
        telemetry.addData("gyro: ", String.valueOf(robot.gyroSensor.getIntegratedZValue()));
        telemetry.update();
        stop(100);
        timer.reset();
        while ((robot.gyroSensor.getIntegratedZValue() > 180)&&(timer.seconds()< 2.55)) {
            robot.motorRightFront.setPower(.3);
            robot.motorLeftFront.setPower(-.3);
            robot.motorRightBack.setPower(.3);
            robot.motorLeftBack.setPower(-.3);
            telemetry.addData("gyro", "X:" + robot.gyroSensor.rawX() / 100 + " Y:" + robot.gyroSensor.rawY() / 100 + " Z:" + robot.gyroSensor.rawZ() / 100);
            telemetry.addData("gyroHead", String.valueOf(robot.gyroSensor.getIntegratedZValue()));
            telemetry.update();
        }
    }

    public int getColor(){
        int dec = -0;
        robot.verticalArm.setPosition(0.05);
        sleep(200);
        double[] colors = new double[150];
        for(int count = 0; count < 3; count++){
            for(int i = 0; i < colors.length; i++){
                colors[i] = robot.colorSensor.colorNumber();
            }
            if((dec = decideColor(colors)) != BLACK) return dec;
            else robot.verticalArm.setPosition(robot.verticalArm.getPosition()- 0.01);
        }
        return BLACK;
    }

    public int decideColor(double[] c){
        int[] colorNums = {0,0,0};
        for(int i = 0; i < c.length; i++){
            if(c[i] >= 9) colorNums[0]++; // Red Reading
            else if(c[i] <= 4 && c[i] != 0) colorNums[1]++; //Blue Reading
            else colorNums[2]++; // Blacks == not reading anything
        }
        telemetry.addData("colors0: ",colorNums[0]);
        telemetry.addData("colors1: ",colorNums[1]);
        telemetry.addData("colors2: ",colorNums[2]);
        telemetry.update();
        sleep(1000);
        switch(findMaxIndex(colorNums)){
            case(0):
                return RED;
            case(1):
                return BLUE;
            case(2):
                return BLACK;
            default:
                return BLACK;
        }
    }

    public void blueHitJewel(){
        robot.horizontalArm.setPosition(0.41);
        stop(50);
        int ballColor = getColor();
        sleep(50);
        robot.verticalArm.setPosition(0);
        sleep(50);
        if(ballColor == RED) hitForward();
        else if(ballColor == BLUE)hitBackward();
        else{ robot.verticalArm.setPosition(0.68); sleep(300) ;}
    }

    public void redHitJewel(){
        robot.verticalArm.setPosition(0.3);
        sleep(300);
        robot.horizontalArm.setPosition(0.39);
        stop(50);
        int ballColor = getColor();
        sleep(50);
        robot.verticalArm.setPosition(0);
        sleep(50);
        if(ballColor == BLUE) {hitForward();}
        else if(ballColor == RED) {hitBackward();}
        else{ robot.verticalArm.setPosition(0.68); sleep(300) ;}
    }

    public int findMaxIndex(int[] c){
        int max = -2147483648;
        int index = -1;
        for(int i = 0; i < c.length; i++){
            if(c[i] > max){
                max = c[i];
                index = i;
            }
        }
        return index;
    }

    public double getRightDist(){
        double reading = 0;
        double cur = -1;
        for(int i = 0; i < 2; i++){
            while((cur = robot.rangeSensor1.getDistance(DistanceUnit.CM)) > 70){}
            reading += cur;
            sleep(1);
        }
        return reading/2;
    }

    public double getLeftDist(){
        double reading = 0;
        double cur = -1;
        for(int i = 0; i < 2; i++){
            while((cur = robot.rangeSensor2.getDistance(DistanceUnit.CM)) > 70){}
            reading += cur;
            sleep(1);
        }
        return reading/2;
    }

    public double getWallDistance(){
        double temp = 0;
        double ran1 = robot.rangeSensor1.getDistance(DistanceUnit.CM);
        double ran2 = robot.rangeSensor2.getDistance(DistanceUnit.CM);
        boolean det = true;
        int count = 0;
        for(int i = 0; i < 10; i++){
            while(det == true) {
                if ((ran1 = robot.rangeSensor1.getDistance(DistanceUnit.CM)) > 70 || (ran2 = robot.rangeSensor2.getDistance(DistanceUnit.CM)) > 70) {
                    ;
                }
                else {break;}
            }
            temp += (ran1 + ran2) / 2;
        }
        return distanceToWall = temp / 10;
    }

    public boolean detectRightEdge(){
        if(distanceToWall < 0) getWallDistance();
        sleep(50);
        double cur = getRightDist();
        if(cur <= 9) return true;
        return false;
    }

    public boolean detectLeftEdge(){
        if(distanceToWall < 0) getWallDistance();
        sleep(50);
        double cur = getLeftDist();
        if(cur <= 9) return true;
        return false;
    }

    public void DriveToColumnBlue(int vu){
        int count = -1;
        while(count != vu){
            robot.motorRightFront.setPower(0.2);
            robot.motorLeftFront.setPower(-0.2);
            robot.motorRightBack.setPower(-0.2);
            robot.motorLeftBack.setPower(.2);
            if(detectLeftEdge()) count++;
        }
        stop(100);
    }

    public int redVuConvert(int vu){
        switch(vu){
            case(0): return 2;
            case(1): return 1;
            case(2): return 0;
            default: return 0;
        }
    }

    public void DriveToColumnRed(int vu){
        int count = -1;
        vu = redVuConvert(vu);
        while(count != vu && opModeIsActive()){
            robot.motorRightFront.setPower(-0.15);
            robot.motorLeftFront.setPower(0.15);
            robot.motorRightBack.setPower(0.15);
            robot.motorLeftBack.setPower(-0.15);
            if(detectRightEdge()) count++;
        }
        stop(100);
    }

    public void bluePutBlock(int vu){
        DriveToColumnBlue(vu);
        encoderDrive(0.2,-0.2,-0.2,10);
        stop(100);
        dump();
    }

    public void redPutBlock(int vu){
        DriveToColumnRed(vu);
        encoderDrive(0.2,-0.2,-0.2,10);
        stop(100);
        dump();
    }

    public void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS) {
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;
        robot.motorLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorLeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorRightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = robot.motorLeftFront.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightFrontTarget = robot.motorRightFront.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            newLeftBackTarget = robot.motorLeftBack.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightBackTarget = robot.motorRightBack.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            robot.motorLeftFront.setTargetPosition(newLeftFrontTarget);
            robot.motorRightFront.setTargetPosition(newRightFrontTarget);
            robot.motorLeftBack.setTargetPosition(newLeftBackTarget);
            robot.motorRightBack.setTargetPosition(newRightBackTarget);

            // Turn On RUN_TO_POSITION
            robot.motorLeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorRightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorLeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorRightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            timer.reset();
            robot.motorLeftFront.setPower(Math.abs(speed));
            robot.motorRightFront.setPower(Math.abs(speed));
            robot.motorLeftBack.setPower(Math.abs(speed));
            robot.motorRightBack.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will sleep.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (timer.seconds() < timeoutS) &&
                    (robot.motorLeftFront.isBusy() && robot.motorRightFront.isBusy()
                            && robot.motorLeftBack.isBusy() && robot.motorRightBack.isBusy() )) {
            }

            // sleep all motion;
            robot.motorLeftFront.setPower(Math.abs(0));
            robot.motorRightFront.setPower(Math.abs(0));
            robot.motorLeftBack.setPower(Math.abs(0));
            robot.motorRightBack.setPower(Math.abs(0));
            // Turn off RUN_TO_POSITION
            robot.motorLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorLeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorRightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    public int Identification(){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = "ATk/k+3/////AAAAGQVxFjsA3E+LryZtRvK0qokfrwD8KdrqkYPWHciWcxQoxYRZbBBeOct2cWmnoSkYw38zfm3veUqW1dTIJL1VbLdss3mUKRx5xJa5eaJN4j2hRq3JGmO2F2NdlbYoEs1trA6MpeWCzm9LHmk7IygaP7b4Xk0DtBeehMLwkN8mWnryA1KSPt3z3JsGvQr1jZ3frJLVJVY7a8rEGaL8gPpERNAXVGlx5BXBOQfoDjpXsGdj89iIJibd9TdIAAR0fkxFMFj27uOMdFk+MtmUmEAI0edmkk46zbAfdLpjkOSCg3JzTwpQPdXksQhlqDQZOr0uyQgU8KaJakPhHsXFiRd8OujwFZ3xXVnPccy/1KCIoQNb";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTrackables.activate();
        timer.reset();
        while (opModeIsActive() && (timer.seconds() < 4.0)) {
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) { // compare the current vumark stream with the built-in sample
                telemetry.addData("VuMark", "%s visible", vuMark);
                vuMarkVal = vuMark.toString();
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();
                telemetry.addData("Pose", format(pose));
                if (pose != null) {
                    VectorF trans = pose.getTranslation();
                    Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
                    double tX = trans.get(0);
                    double tY = trans.get(1);
                    double tZ = trans.get(2);

                    double rX = rot.firstAngle;
                    double rY = rot.secondAngle;
                    double rZ = rot.thirdAngle;
                    if(vuMarkVal.equals("RIGHT")) return 0;
                    if(vuMarkVal.equals("CENTER")) return 1;
                    if(vuMarkVal.equals("LEFT")) return 2;
                }
            }
            else {
                telemetry.addData("VuMark", "not visible");
            }
            telemetry.update();
        }
        return 0;
    }

    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }

    public void gyroDrive ( double speed, double distance, double angle) {
        int     newLeftFrontTarget;
        int     newRightFrontTarget;
        int     newLeftBackTarget;
        int     newRightBackTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH);
            newLeftFrontTarget = robot.motorLeftFront.getCurrentPosition() + moveCounts;
            newRightFrontTarget = robot.motorRightFront.getCurrentPosition() + moveCounts;
            newLeftBackTarget = robot.motorLeftBack.getCurrentPosition() + moveCounts;
            newRightBackTarget = robot.motorRightBack.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            robot.motorLeftFront.setTargetPosition(newLeftFrontTarget);
            robot.motorRightFront.setTargetPosition(newRightFrontTarget);
            robot.motorLeftBack.setTargetPosition(newLeftBackTarget);
            robot.motorRightBack.setTargetPosition(newRightBackTarget);

            robot.motorLeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorRightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorLeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorRightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            robot.motorLeftFront.setPower(speed);
            robot.motorRightFront.setPower(speed);
            robot.motorLeftBack.setPower(speed);
            robot.motorRightBack.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() && (robot.motorLeftFront.isBusy() && robot.motorRightFront.isBusy()
                    && robot.motorLeftBack.isBusy() && robot.motorRightBack.isBusy())) {
                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                robot.motorLeftBack.setPower(leftSpeed);
                robot.motorLeftFront.setPower(leftSpeed);
                robot.motorRightFront.setPower(rightSpeed);
                robot.motorRightBack.setPower(rightSpeed);

                // Display drive status for the driver.
                /*
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d",      newLeftTarget,  newRightTarget);
                telemetry.addData("Actual",  "%7d:%7d",      robot.leftDrive.getCurrentPosition(),
                        robot.rightDrive.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                telemetry.update();
                */
            }

            // sleep all motion;
            robot.motorLeftFront.setPower(0);
            robot.motorRightFront.setPower(0);
            robot.motorLeftBack.setPower(0);
            robot.motorRightBack.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.motorLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorLeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorRightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will sleep if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver sleeps the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroTurn (double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }

    /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will sleep once the requested time has elapsed
     *
     * @param speed      Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    public void gyroHold( double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
        }

        // sleep all motion;
        robot.motorLeftFront.setPower(0);
        robot.motorRightFront.setPower(0);
        robot.motorLeftBack.setPower(0);
        robot.motorRightBack.setPower(0);
    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        robot.motorLeftFront.setPower(leftSpeed);
        robot.motorRightFront.setPower(rightSpeed);
        robot.motorLeftBack.setPower(leftSpeed);
        robot.motorRightBack.setPower(rightSpeed);


        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - robot.gyroSensor.getIntegratedZValue();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    public void blueStrategyShort() {
        while (!opModeIsActive() || robot.gyroSensor.isCalibrating()) {
        }
        try {
            vuMarkValIn = Identification();
        } catch (Exception e) {
            vuMarkValIn = 0;
        }
        blueHitJewel();
        stop(50);
        encoderDrive(0.4, 9.5, 9.5, 10);
        /*stop(100);
        turnRight90Blue();
        encoderDrive(0.3, -1.5, -1.5, 10);
        DriveToColumnBlue(vuMarkValIn);
        stop(10);
        dump();
        */
    }

    public void redStrategyShort(){
        while(!opModeIsActive() || robot.gyroSensor.isCalibrating()){
        }
        if(opModeIsActive()) {
            /*
            try {
                vuMarkValIn = Identification();
            } catch (Exception e) {
                vuMarkValIn = 2;
            }
            redHitJewel();
            stop(50);
            */
            encoderDrive(0.4, -8.2, -8.2, 10);
            stop(100);
            turnRight90Red();
            stop(10);
            backToEndRed(20);
            DriveToColumnRed(0);
            stop(10);
            dump();

        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            setup();
            robot.motorLeftFront.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
            robot.motorRightFront.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
            robot.motorLeftBack.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
            robot.motorRightBack.setDirection(DcMotor.Direction.FORWARD);
            blueStrategyShort();
        }catch(Exception e){
            stop(10);
        }
    }
}



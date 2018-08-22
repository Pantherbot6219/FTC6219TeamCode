package org.firstinspires.ftc.teamcode.Team6219Gen5Code;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.Hardware.MRColorSensor;
/**
 * Created by KusakabeMirai on 9/13/2017.
 */

/**
 * The class is created for better initializations, and easier understanding and implementation.
 * Improves efficiency marginally
 * All robots part should be defined as public objects
 */
// this class is an extension of the pantherbot6219
// for robot that implements a mecanum/omniwheel drive system

public class Gen5Hardware {
    /* Public OpMode members. */
    public DcMotor motorLeftBack                    = null;
    public DcMotor motorLeftFront                   = null;
    public DcMotor motorRightBack                   = null;
    public DcMotor motorRightFront                  = null;
    public DcMotor slide                            = null;
    public DcMotor lift                             = null;
    public DcMotor leftIntake                       = null;
    public DcMotor rightIntake                      = null;
    public Servo verticalArm                        = null;
    public Servo horizontalArm                      = null;
    public Servo claw                               = null;
    public Servo joint                              = null;
    public Servo dump1                              = null;
    public Servo dump2                              = null;
    public I2cDevice colorReader                    = null;
    public MRColorSensor colorSensor                = null;
    public ModernRoboticsI2cRangeSensor rangeSensor1= null;
    public ModernRoboticsI2cRangeSensor rangeSensor2= null;
    public ModernRoboticsI2cRangeSensor rangeSensor3= null;
    public OpticalDistanceSensor distanceSensor1    = null;
    public OpticalDistanceSensor distanceSensor2    = null;
    public ModernRoboticsI2cGyro gyroSensor         = null;
    public static int teamColor                     =  1;

    HardwareMap hwMap                               = null;
    private ElapsedTime period                      = new ElapsedTime();


    public Gen5Hardware(HardwareMap ahwMap, int teamColor){
        init(ahwMap,teamColor);
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap,int color) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors, Servos, and Sensors.
        this.teamColor = color;
        motorLeftFront = hwMap.dcMotor.get("motorLeftFront");
        motorRightFront= hwMap.dcMotor.get("motorRightFront");
        motorLeftBack  = hwMap.dcMotor.get("motorLeftBack");
        motorRightBack = hwMap.dcMotor.get("motorRightBack");
        slide          = hwMap.dcMotor.get("slide");
        lift           = hwMap.dcMotor.get("lift");
        leftIntake     = hwMap.dcMotor.get("leftIntake");
        rightIntake    = hwMap.dcMotor.get("rightIntake");
        claw           = hwMap.servo.get("claw");
        joint          = hwMap.servo.get("joint");
        verticalArm    = hwMap.servo.get("varm");
        horizontalArm  = hwMap.servo.get("harm");
        dump1          = hwMap.servo.get("dumpLeft");
        dump2          = hwMap.servo.get("dumpRight");
        colorReader    = hwMap.i2cDevice.get("color");
        colorSensor    = new MRColorSensor(colorReader.getI2cController(), colorReader.getPort());
        //distanceSensor1= hwMap.opticalDistanceSensor.get("d1"); //a0 is detect intake
        //distanceSensor2= hwMap.opticalDistanceSensor.get("d2"); //a1 is also detect intake
        gyroSensor     = (ModernRoboticsI2cGyro)hwMap.gyroSensor.get("gyro");
        rangeSensor1   = hwMap.get(ModernRoboticsI2cRangeSensor.class, "r1"); // right
        rangeSensor2   = hwMap.get(ModernRoboticsI2cRangeSensor.class, "r2"); // left
        //rangeSensor3   = hwMap.get(ModernRoboticsI2cRangeSensor.class, "r3");

        colorSensor.setI2cAddress(I2cAddr.create8bit(0x3c)); //LEFT
        rangeSensor1.setI2cAddress(I2cAddr.create8bit(0x28)); //Right
        rangeSensor2.setI2cAddress(I2cAddr.create8bit(0x26)); //Left
        //rangeSensor3.setI2cAddress(I2cAddr.create8bit(0x24)); //Intake

        motorLeftFront.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        motorRightFront.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        motorLeftBack.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        motorRightBack.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        lift.setDirection(DcMotor.Direction.FORWARD);
        leftIntake.setDirection(DcMotor.Direction.REVERSE);
        rightIntake.setDirection(DcMotor.Direction.FORWARD);
        /*
        joint.setDirection(Servo.Direction.REVERSE);
        claw.setDirection(Servo.Direction.REVERSE);
        */
        // Set all motors' encoders.
        // Even if some motors are indeed encoder motors,
        // As long as the functionality of the encoder part is not used,
        // Using RUN_WITHOUT_ENCODER will improve stability and efficiency.
        motorLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        claw.setPosition(1);
        joint.setPosition(0);
        verticalArm.setPosition(0.67);
        horizontalArm.setPosition(0.45);
        dump1.setDirection(Servo.Direction.REVERSE);
        dump1.setPosition(0.13);
        dump2.setPosition(0.26);

        // Set all motors to zero power and original position
        motorLeftFront.setPower(0);
        motorRightFront.setPower(0);
        motorLeftBack.setPower(0);
        motorRightBack.setPower(0);
        lift.setPower(0);
        //
        // slide.setPower(0);
        leftIntake.setPower(0);
        rightIntake.setPower(0);

        colorSensor.enableLed(true);

        gyroCalibrate();
    }

    public void gyroCalibrate(){
        this.gyroSensor.calibrate();
        while(this.gyroSensor.isCalibrating()){

        }
        return;
    }

    public void waitForTick(long periodMs) {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0) {
            try {
                Thread.sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        // Reset the cycle clock for the next pass.
        period.reset();
    }

}

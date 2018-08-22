package org.firstinspires.ftc.teamcode.Hardware;

/**
 * Created by KusakabeMirai on 6/19/2018.
 */

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.teamcode.Hallib.HalDashboard;
import org.firstinspires.ftc.teamcode.Hallib.HalUtil;

public class JewelArm {

    private ColorSensor colorSensor;
    private CRServo thrust;
    private Servo kicker;
    private LinearOpMode opMode;
    private ElapsedTime mClock = new ElapsedTime();

    private double servoForward = -.1;
    private double servoBackward = .7;
    private double servoRest = 0;
    private boolean blueAlliance = true;

    public JewelArm (LinearOpMode opMode)
    {
        this.opMode = opMode;
        colorSensor = opMode.hardwareMap.colorSensor.get("colorSensor");
        thrust = opMode.hardwareMap.crservo.get("thrust");
        kicker = opMode.hardwareMap.servo.get("kicker");
        mClock.reset();
    }

    public void pushJewels (boolean blueAlliance){
        colorSensor.enableLed(true);
        if(blueAlliance){
            if(colorSensor.blue() > colorSensor.red() && colorSensor.blue() > colorSensor.green()){
                kicker.setPosition(0);
                HalUtil.sleep(500);
            }
            else if (colorSensor.red() > colorSensor.blue() && colorSensor.red() > colorSensor.green()){
                kicker.setPosition(.5);
                HalUtil.sleep(500);
            }
            else {
                kicker.setPosition(.25);
            }
        } else {
            if (colorSensor.blue() > colorSensor.red() && colorSensor.blue() > colorSensor.green()) {
                kicker.setPosition(.5);
                HalUtil.sleep(500);
            } else if (colorSensor.red() > colorSensor.blue() && colorSensor.red() > colorSensor.green()) {
                kicker.setPosition(0);
                HalUtil.sleep(500);
            } else {
                kicker.setPosition(.25);
            }
        }
        colorSensor.enableLed(false);
    }

    public void armDown() {
        thrust.setPower(1);
        HalUtil.sleep(2500);
        kicker.setPosition(.25);
        HalUtil.sleep(3500);
    }

    public void armUp(){
        thrust.setPower(-1);
        HalUtil.sleep(2500);
        kicker.setPosition(.8);

    }

    public void resetServo()
    {
        kicker.setPosition(0.25);
        colorSensor.enableLed(false);
        thrust.setPower(-.2);
    }

}
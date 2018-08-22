package org.firstinspires.ftc.teamcode.Hardware;

/**
 * Created by KusakabeMirai on 6/19/2018.
 */
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot.RobotMap;

/**
 * Created by margaretli on 12/16/17.
 */

public class RelicArm {
    public DcMotor armextension;
    Servo armgripper;
    Servo claw;
    LinearOpMode opMode;

    public RelicArm(LinearOpMode opMode){
        armextension = opMode.hardwareMap.dcMotor.get(RobotMap.DcMotor.slide);
        armextension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        claw = opMode.hardwareMap.servo.get("claw");
    }

    public void open(){
        claw.setPosition(.7);
    }

    public void close(){
        claw.setPosition(0);
    }

    public void extend(){
    }

    public void retract(){
    }


    public void armExtension (double power){
        armextension.setPower(power);
    }

    public void moveArm (double power){
    }
}
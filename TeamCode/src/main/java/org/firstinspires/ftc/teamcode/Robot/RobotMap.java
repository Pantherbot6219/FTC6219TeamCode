package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.I2cAddr;

/**
 * Created by KusakabeMirai on 7/22/2018.
 */

public class RobotMap {
    public static class DcMotor{
        public static final String
            leftFrontMotor = "motor11",
            rightFrontMotor = "motor21",
            leftBackMotor = "motor12",
            rightBackMotor = "motor22",
            leftIntake = "leftIn",
            rightIntake = "rightIn",
            slide = "slide",
            lift= "lift";
    }

    public static class Servo{
        public static final String
            kicker = "kicker",
            throttler = "throttler",
            joint = "joint",
            claw = "claw";
    }

    public static class I2c{
        public static final String
            colorLeft = "colorL",
            colorRight = "colorR",
            distanceLeft = "distanceL",
            distanceRight = "distanceR",
            gyro = "gyro";
    }

    public static class Analog{

    }

}

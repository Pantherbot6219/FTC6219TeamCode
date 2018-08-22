package org.firstinspires.ftc.teamcode.Hardware;

/**
 * Created by KusakabeMirai on 6/13/2018.
 */
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

/**
 * {@link IMU} gives a short demo on how to use the BNO055 Inertial Motion Unit (IMU) from AdaFruit.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 *
 * @see <a href="http://www.adafruit.com/products/2472">Adafruit IMU</a>
 */
@Autonomous(name = "test BNO055 IMU", group = "Sensor")
public class IMU extends LinearOpMode {

    BNO055IMU IMU;
    Orientation angles;
    Acceleration gravity;

    public IMU (BNO055IMU imu){
        IMU = imu;
    }

    public void IMUinit(HardwareMap map) {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        // parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        IMU = map.get(BNO055IMU.class, "imu");
        IMU.initialize(parameters);


    }

    public double getYaw() { //returns yaw between -179.9999 and 180 degrees
        angles = IMU.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        return -Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
    }

    public double getPitch() { //returns yaw between -179.9999 and 180 degrees
        angles = IMU.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        return Double.parseDouble(formatAngle(angles.angleUnit, angles.thirdAngle));
    }

    public double getRoll() {
        angles = IMU.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        return Double.parseDouble(formatAngle(angles.angleUnit, angles.secondAngle));
    }

    public double getTrueDiff(double origAngle) {
        double currAngle = getYaw();
        if (currAngle >= 0 && origAngle >= 0 || currAngle <= 0 && origAngle <= 0)
            return currAngle - origAngle;
        else if (Math.abs(currAngle - origAngle) <= 180)
            return currAngle - origAngle;
        else if (currAngle > origAngle)
            return -(360 - (currAngle - origAngle));
        else
            return (360 + (currAngle - origAngle));
    }

    String formatAngle(org.firstinspires.ftc.robotcore.external.navigation.AngleUnit angleUnit, double angle) {
        return formatDegrees(org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES.normalize(degrees));
    }

    public void runOpMode() {}

}
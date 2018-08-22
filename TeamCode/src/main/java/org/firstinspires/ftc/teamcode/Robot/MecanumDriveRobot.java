package org.firstinspires.ftc.teamcode.Robot;

/**
 * Created by KusakabeMirai on 6/19/2018.
 */

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hallib.HalDashboard;
import org.firstinspires.ftc.teamcode.Hardware.Intake;
import org.firstinspires.ftc.teamcode.Hardware.JewelArm;
import org.firstinspires.ftc.teamcode.Hardware.Lift;
import org.firstinspires.ftc.teamcode.Hardware.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.Hardware.RelicArm;
import org.firstinspires.ftc.teamcode.Hardware.VuMark;

public class MecanumDriveRobot {

    private static HalDashboard dashboard = null;
    public MecanumDriveBase driveBase = null;
    public JewelArm jewelArm = null;
    public org.firstinspires.ftc.teamcode.Hardware.VuMark VuMark = null;
    public Lift lift = null;
    public Intake intake = null;
    public RelicArm relicArm = null;

    public MecanumDriveRobot(LinearOpMode opMode, boolean auto) throws InterruptedException {
        dashboard = new HalDashboard(opMode.telemetry);
        jewelArm = new JewelArm(opMode);
        driveBase = new MecanumDriveBase(opMode);
        VuMark = new VuMark(opMode);
        lift = new Lift(opMode);
        intake = new Intake(opMode);
        relicArm = new RelicArm(opMode);
    }

    public static HalDashboard getDashboard() {
        return dashboard;
    }

}
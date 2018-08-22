package org.firstinspires.ftc.teamcode.Pantherbot6219OpMode;

/**
 * Created by KusakabeMirai on 7/11/2018.
 */

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.Pantherbot6219Lib.FTCMenu;
import org.firstinspires.ftc.teamcode.Pantherbot6219Lib.FTCChoiceMenu;
import org.firstinspires.ftc.teamcode.Robot.TankDriveRobot;

@Autonomous(name="PantherbotAutonomous", group="Autonomous")
public class Team6219Autonomous extends LinearOpMode implements FTCMenu.MenuButtons{
    private TankDriveRobot robot;

    private ElapsedTime mClock = new ElapsedTime();

    //different scenerios
    public enum Alliance_Position {
        BLUE_RIGHT,
        BLUE_LEFT,
        RED_RIGHT,
        RED_LEFT
    }

    //defaults to blue right
    private Alliance_Position alliance = Alliance_Position.BLUE_RIGHT;
    //defaults to center vumark
    private RelicRecoveryVuMark vumark = RelicRecoveryVuMark.CENTER;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new TankDriveRobot(this, true);// need this to run menu test
        doMenus();
        //scan and read picture
        robot.VuMark.activate();
        waitForStart();
        // robot.driveBase.resetHeading();
        if (alliance == Alliance_Position.BLUE_RIGHT) {
            //to mark original heading
            robot.driveBase.resetIntZ();
            robot.VuMark.decodePictograph();
            telemetry.addData("VuMark", "%s visible", robot.VuMark.getCryptoboxKey());
            telemetry.update();
            vumark = (robot.VuMark.getCryptoboxKey());
            //jewel arm
            robot.jewelArm.armDown();
            vumark = (robot.VuMark.getCryptoboxKey());
            robot.jewelArm.pushJewels(true);
            robot.jewelArm.armUp();
            //drive to cryptobox and place glyph
            if (vumark == RelicRecoveryVuMark.LEFT) {
                robot.driveBase.drivePID(31, false);
            } else if (vumark == RelicRecoveryVuMark.RIGHT) {
                robot.driveBase.drivePID(47 , false);
            } else {
                robot.driveBase.drivePID(39, false);
            }
            robot.driveBase.spinPID(-90);
            robot.driveBase.drivePID(15, false);
            robot.intake.out();
            robot.driveBase.drivePID(-8,false);
            robot.intake.stop();
            /*
            robot.driveBase.spinPID(180);
            robot.intake.in(.8);
            robot.driveBase.drivePID(18,false);
            robot.driveBase.spinPID(-180);
            robot.driveBase.drivePID(32,false);
            robot.intake.out();
            robot.driveBase.drivePID(-8,false);
            robot.intake.stop();
            */
        }
        else if (alliance == Alliance_Position.BLUE_LEFT) {
            //to mark original heading
            robot.driveBase.resetIntZ();
            robot.VuMark.decodePictograph();
            telemetry.addData("VuMark", "%s visible", robot.VuMark.getCryptoboxKey());
            telemetry.update();
            vumark = (robot.VuMark.getCryptoboxKey());
            //jewel arm
            robot.jewelArm.armDown();
            robot.jewelArm.pushJewels(true);
            robot.jewelArm.armUp();
            //drive to cryptobox and place glyph
            if (vumark == RelicRecoveryVuMark.LEFT) {
                robot.driveBase.drivePID(25, false);
                robot.driveBase.spinPID(15);
                robot.driveBase.drivePID(11, false);
            } else if (vumark == RelicRecoveryVuMark.RIGHT) {
                robot.driveBase.drivePID(26, false);
                robot.driveBase.spinPID(90);
                robot.driveBase.drivePID(22, false);
                robot.driveBase.spinPID(-90);
                robot.driveBase.drivePID(6, false);
            } else {
                robot.driveBase.drivePID(26, false);
                robot.driveBase.spinPID(90);
                robot.driveBase.drivePID(16, false);
                robot.driveBase.spinPID(-90);
                robot.driveBase.drivePID(6, false);
            }
            robot.intake.out();
            robot.driveBase.drivePID(-6,false);
            robot.intake.stop();
        }
        else if (alliance == Alliance_Position.RED_RIGHT) {
            //to mark original heading
            robot.driveBase.resetIntZ();
            robot.VuMark.decodePictograph();
            telemetry.addData("VuMark", "%s visible", robot.VuMark.getCryptoboxKey());
            telemetry.update();
            vumark = (robot.VuMark.getCryptoboxKey());
            //jewel arm
            robot.jewelArm.armDown();
            robot.jewelArm.pushJewels(false);
            robot.jewelArm.armUp();
            //drive to cryptobox and place glyph
            if (vumark == RelicRecoveryVuMark.LEFT) {
                robot.driveBase.drivePID(-26, false);
                robot.driveBase.spinPID(90);
                robot.driveBase.drivePID(18, false);
                robot.driveBase.spinPID(80);
                robot.driveBase.drivePID(10, false);
            } else if (vumark == RelicRecoveryVuMark.RIGHT) {
                robot.driveBase.drivePID(-24, false);
                robot.driveBase.spinPID(-170);
                robot.driveBase.drivePID(5, false);

            } else {
                robot.driveBase.drivePID(-26, false);
                robot.driveBase.spinPID(90);
                robot.driveBase.drivePID(12, false);
                robot.driveBase.spinPID(80);
                robot.driveBase.drivePID(10, false);
            }
            robot.intake.out();
            robot.driveBase.drivePID(-6,false);
            robot.intake.stop();
        }
        else if (alliance == Alliance_Position.RED_LEFT) {
            //to mark original heading
            robot.driveBase.resetIntZ();
            robot.VuMark.decodePictograph();
            telemetry.addData("VuMark", "%s visible", robot.VuMark.getCryptoboxKey());
            telemetry.update();
            vumark = (robot.VuMark.getCryptoboxKey());
            //jewel arm
            robot.jewelArm.armDown();
            robot.jewelArm.pushJewels(false);
            robot.jewelArm.armUp();
            //drive to cryptobox and place glyph
            if (vumark == RelicRecoveryVuMark.LEFT) {
                robot.driveBase.drivePID(-40, false);
            }
            else if (vumark == RelicRecoveryVuMark.RIGHT){
                robot.driveBase.drivePID(-26, false);
            }
            else {
                robot.driveBase.drivePID(-34, false);
            }
            robot.driveBase.spinPID(-90);
            robot.driveBase.drivePID(13, false);
            robot.intake.out();
            robot.driveBase.drivePID(-7,false);
            robot.intake.stop();
        }
        runCleanUp();
        telemetry.clearAll();
        telemetry.update();
    }

    private void runCleanUp() throws InterruptedException {
        robot.driveBase.resetMotors();
        robot.driveBase.resetPIDDrive();
    }

    //menu
    @Override
    public boolean isMenuUpButton() {
        return gamepad1.dpad_up;
    }

    @Override
    public boolean isMenuDownButton() {
        return gamepad1.dpad_down;
    }

    @Override
    public boolean isMenuEnterButton() {
        return gamepad1.dpad_right;
    }

    @Override
    public boolean isMenuBackButton() {
        return gamepad1.dpad_left;
    }

    private void doMenus() throws InterruptedException {
        FTCChoiceMenu alliancePositionMenu = new FTCChoiceMenu("Alliance Position:", null, this);

        alliancePositionMenu.addChoice("BLUE RIGHT", Team6219Autonomous.Alliance_Position.BLUE_RIGHT);
        alliancePositionMenu.addChoice("BLUE LEFT", Team6219Autonomous.Alliance_Position.BLUE_LEFT);
        alliancePositionMenu.addChoice("RED RIGHT", Team6219Autonomous.Alliance_Position.RED_RIGHT);
        alliancePositionMenu.addChoice("RED LEFT", Team6219Autonomous.Alliance_Position.RED_LEFT);

        FTCMenu.walkMenuTree(alliancePositionMenu);
        alliance = (Team6219Autonomous.Alliance_Position) alliancePositionMenu.getCurrentChoiceObject();
    }

}
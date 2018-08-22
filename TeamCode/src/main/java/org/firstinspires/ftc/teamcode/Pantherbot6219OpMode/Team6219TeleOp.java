package org.firstinspires.ftc.teamcode.Pantherbot6219OpMode;

/**
 * Created by KusakabeMirai on 7/31/2018.
 */

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot.MecanumDriveRobot;

import static java.lang.Math.abs;
import org.firstinspires.ftc.teamcode.Pantherbot6219Lib.*;

@TeleOp(name = "LockdownTeleOp", group = "TeleOp")
//@Disabled

public class Team6219TeleOp extends LinearOpMode {

    private MecanumDriveRobot robot;
    private boolean relicMode = false;
    private boolean invertDrive = false;

    public void runOpMode() throws InterruptedException {
        robot = new MecanumDriveRobot(this, false);
        robot.driveBase.noEncoders();
        robot.jewelArm.resetServo();
        robot.driveBase.slowSpeed(false);
        waitForStart();
        while (opModeIsActive()) {
            //drive train
            if (gamepad1.b) {
                robot.driveBase.slowSpeed(true);
            }
            if (gamepad1.x){
                invertDrive = true;
            }
            else if (gamepad1.a) {
                robot.driveBase.slowSpeed(false);
                invertDrive = false;
            }
            if (!invertDrive){
                robot.driveBase.tankDrive(gamepad1.left_stick_y, gamepad1.right_stick_y);
            }
            else if (invertDrive){
                robot.driveBase.tankDrive(-gamepad1.left_stick_y, -gamepad1.right_stick_y);
            }

            //intake
            if (gamepad1.right_bumper){
                robot.intake.out();
            }
            else if (gamepad1.right_trigger > .6){
                robot.intake.in(.8);
            }
            else{
                robot.intake.stop();
            }
            //relic mode toggle
            if (!relicMode && gamepad2.x){
                relicMode = true;
            }
            else {
                relicMode = false;
            }
            //relic arm
            if (relicMode) {
                robot.relicArm.armExtension(-gamepad2.left_stick_y);
                if (abs(gamepad2.right_stick_y) > .2) {
                    robot.relicArm.moveArm(-gamepad2.right_stick_y);
                }
                if (gamepad2.left_bumper) {
                    robot.relicArm.open();
                }
                if (gamepad2.right_bumper) {
                    robot.relicArm.close();
                }
                if (gamepad2.left_trigger > 0.6) {
                    robot.relicArm.extend();
                }
                if (gamepad2.right_trigger > 0.6) {
                    robot.relicArm.retract();
                }

            }
            else {
                //lift
                robot.lift.control(-gamepad2.right_stick_y);
            }
            if(robot.lift.isTargetReached);
            else robot.lift.liftController(0,true);

        }

    }
}

package org.firstinspires.ftc.teamcode.AutoOp;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.modules.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.modules.drive.SampleMecanumDrive;
//import org.firstinspires.ftc.teamcode.RobotCore;
import org.firstinspires.ftc.teamcode.modules.trajectorysequence.TrajectorySequence;

import java.util.Arrays;


@Config
@Autonomous(name = "Robotblazers auto path test", group = "23692")
public class AutonTest extends LinearOpMode {

    boolean finished = false;
    public SampleMecanumDrive drive;
    @Override
    public void runOpMode() {
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        TrajectoryVelocityConstraint slowCont = new MinVelocityConstraint(Arrays.asList(
                new TranslationalVelocityConstraint(0.2),
                new AngularVelocityConstraint(1)));

        /* Notes:
        1. 90 deg is facing red
        2. 0 deg is facing backdrop
        3. -x toward audience
        4. +x towards backdrop
        5. -y towards red
        6. +y towards blue
         */

        //RobotCore robot = new RobotCore(hardwareMap);
        TrajectorySequence untitled01 = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(0)))// 90 is facing red
                //.lineTo(new Vector2d(36, 0))
                //.lineTo(new Vector2d(-36.00, -20.00))
                //.lineTo(new Vector2d(-50.00, -37.00))
                //.lineTo(new Vector2d(-50, -7.00))
                //.turn(Math.toRadians(-90)) //change after tuning to -90
                //.lineTo(new Vector2d(10, -7))

                .forward(37)
                .back(10)
                .strafeRight(15)
                .forward(28)
                .turn(Math.toRadians(90))
                .strafeLeft(2)
                .back(72)
                .strafeRight(26)
                .back(30)
                //.turn(180)
                .build();

        TrajectorySequence leftSide = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(0)))

                .forward(25) //1
                .turn(Math.toRadians(-90))
                .strafeRight(6)
                .forward(14) //2 L
                .back(16) // 3
                .turn(Math.toRadians(90))
                .strafeRight(10)
                .forward(31)
                .turn(Math.toRadians(90))
                .strafeLeft(2)
                .back(72)
                .strafeRight(26)
                .back(32)
                .build();



        TrajectorySequence untitled0 = drive.trajectorySequenceBuilder(new Pose2d(-36.04, 62.10, Math.toRadians(270.00)))
                .splineTo(new Vector2d(-35.87, 29), Math.toRadians(-89.71))
                .lineTo(new Vector2d(-36.04, 45.42))
                .splineTo(new Vector2d(-48, 40.73), Math.toRadians(267.61))
                .splineTo(new Vector2d(-50, 27.45), Math.toRadians(269.38))
                .splineTo(new Vector2d(-0.26, 12.55), Math.toRadians(-3.18))
                .splineTo(new Vector2d(31.87, 11.20), Math.toRadians(0.61))
                .splineTo(new Vector2d(49.94, 34.13), Math.toRadians(0.66))
                .build();
        TrajectorySequence untitled1 = drive.trajectorySequenceBuilder(new Pose2d(-36.56, 63.70, Math.toRadians(-89.30)))
                .splineTo(new Vector2d(-28.84, 32.55), Math.toRadians(-4.64))
                .splineTo(new Vector2d(-46.79, 29.73), Math.toRadians(230.87))
                .splineTo(new Vector2d(-41.01, 12.53), Math.toRadians(1.12))
                .splineTo(new Vector2d(32.55, 13.87), Math.toRadians(90.00))
                .splineTo(new Vector2d(44.12, 36.26), Math.toRadians(178.60))
                .build();
        TrajectorySequence untitled2 = drive.trajectorySequenceBuilder(new Pose2d(-35.52, 66.96, Math.toRadians(-86.73)))
                .splineTo(new Vector2d(-42.93, 31.51), Math.toRadians(183.95))
                .lineToSplineHeading(new Pose2d(-33.29, 6.30, Math.toRadians(0.00)))
                .splineTo(new Vector2d(32.85, 2.60), Math.toRadians(69.15))
                .splineTo(new Vector2d(43.23, 33.89), Math.toRadians(187.31))
                .build();















        while(!isStopRequested() && !opModeIsActive()) {

        }
        waitForStart();
        if (isStopRequested()) return;

        drive.setPoseEstimate(untitled2.start());

        drive.followTrajectorySequence(untitled2);
    }

}
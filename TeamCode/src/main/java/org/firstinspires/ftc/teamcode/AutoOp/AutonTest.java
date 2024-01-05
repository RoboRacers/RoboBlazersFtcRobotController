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
import org.firstinspires.ftc.teamcode.modules.trajectorysequence.TrajectorySequenceBuilder;

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


//        TrajectorySequence untitled2 = drive.trajectorySequenceBuilder(new Pose2d(-38.48, 65.77, Math.toRadians(264.99)))
//                .splineTo(new Vector2d(-41.75, 33.15), Math.toRadians(180.00))
//                .splineTo(new Vector2d(-39.08, 13.57), Math.toRadians(0.00))
//                .splineTo(new Vector2d(44.42, 7.64), Math.toRadians(88.85))
//                .splineTo(new Vector2d(45.60, 39.67), Math.toRadians(3.81))
//                .build();
//
//        TrajectorySequence untitled0 = drive.trajectorySequenceBuilder(new Pose2d(-36.04, 62.10, Math.toRadians(270.00)))
//                .splineTo(new Vector2d(-35.87, 29), Math.toRadians(-89.71))
//                .lineTo(new Vector2d(-36.04, 45.42))
//                .splineTo(new Vector2d(-48, 40.73), Math.toRadians(267.61))
//                .splineTo(new Vector2d(-50, 27.45), Math.toRadians(269.38))
//                .splineTo(new Vector2d(-0.26, 12.55), Math.toRadians(-3.18))
//                .splineTo(new Vector2d(31.87, 11.20), Math.toRadians(0.61))
//                .splineTo(new Vector2d(49.94, 34.13), Math.toRadians(0.66))
//                .build();
//        TrajectorySequence untitled1 = drive.trajectorySequenceBuilder(new Pose2d(-33.74, 66.66, Math.toRadians(265.24)))
//                .splineTo(new Vector2d(-28.25, 31.81), Math.toRadians(3.58))
//                .splineTo(new Vector2d(-27.81, 42.19), Math.toRadians(171.25))
//                .splineTo(new Vector2d(-46.94, 44.71), Math.toRadians(261.98))
//                .splineTo(new Vector2d(-28.55, -0.07), Math.toRadians(15.15))
//                .splineTo(new Vector2d(37.00, 25.14), Math.toRadians(92.20))
//                .splineTo(new Vector2d(47.83, 37.45), Math.toRadians(174.22))
//                .build();
//                         ^^ Not working paths
        TrajectorySequence BlueFarLeft = drive.trajectorySequenceBuilder(new Pose2d(-35.81, 66.51, Math.toRadians(90.00)))
                .lineToSplineHeading(new Pose2d(-35.52, 33.74, Math.toRadians(180.00)))
                .lineToConstantHeading(new Vector2d(-18.46, 33.74))
                .lineToSplineHeading(new Pose2d(53.17, 33.59, Math.toRadians(0.00)))
                .build();
        TrajectorySequence BlueFarRight = drive.trajectorySequenceBuilder(new Pose2d(-36.26, 67.85, Math.toRadians(89.16)))
                .lineToSplineHeading(new Pose2d(-43.82, 31.81, Math.toRadians(176.53)))
                .lineToSplineHeading(new Pose2d(-28.25, 36.11, Math.toRadians(1.91)))
                .splineToConstantHeading(new Vector2d(53.17, 34.48), Math.toRadians(0.00))
                .build();
        TrajectorySequence BlueFarCenter = drive.trajectorySequenceBuilder(new Pose2d(-35.96, 65.47, Math.toRadians(90.00)))
                .lineToLinearHeading(new Pose2d(-35.52, 19.65, Math.toRadians(84.64)))
                .lineToSplineHeading(new Pose2d(25.29, 9.57, Math.toRadians(164.98)))
                .lineToSplineHeading(new Pose2d(48.87, 33.15, Math.toRadians(177.88)))
                .build();
        TrajectorySequence BlueCloseLeft = drive.trajectorySequenceBuilder(new Pose2d(11.94, 65.92, Math.toRadians(90.00)))
                .lineToSplineHeading(new Pose2d(10.90, 32.11, Math.toRadians(177.66)))
                .splineToConstantHeading(new Vector2d(25.73, 32.11), Math.toRadians(180.00))
                .lineToConstantHeading(new Vector2d(52.72, 31.51))
                .build();
        TrajectorySequence BlueCloseCenter = drive.trajectorySequenceBuilder(new Pose2d(11.20, 61.77, Math.toRadians(87.71)))
                .lineToConstantHeading(new Vector2d(11.35, 20.24))
                .lineToSplineHeading(new Pose2d(47.68, 32.11, Math.toRadians(0.00)))
                .build();
        TrajectorySequence BlueCloseRight = drive.trajectorySequenceBuilder(new Pose2d(12.23, 63.10, Math.toRadians(86.42)))
                .splineToSplineHeading(new Pose2d(4.97, 32.11, Math.toRadians(174.96)), Math.toRadians(174.96))
                .lineToSplineHeading(new Pose2d(50.50, 32.55, Math.toRadians(-1.06)))
                .build();
        TrajectorySequence RedFarCenter = drive.trajectorySequenceBuilder(new Pose2d(-36.41, -67.85, Math.toRadians(268.73)))
                .lineToConstantHeading(new Vector2d(-36.56, -18.91))
                .lineToSplineHeading(new Pose2d(-18.61, -4.97, Math.toRadians(-11.98)))
                .splineTo(new Vector2d(49.75, -37.00), Math.toRadians(3.37))
                .build();
        TrajectorySequence RedFarRight = drive.trajectorySequenceBuilder(new Pose2d(-35.67, -69.33, Math.toRadians(265.87)))
                .lineToSplineHeading(new Pose2d(-35.07, -35.67, Math.toRadians(180.00)))
                .lineToConstantHeading(new Vector2d(-18.61, -35.96))
                .lineToSplineHeading(new Pose2d(53.91, -36.85, Math.toRadians(-1.36)))
                .build();

















        while(!isStopRequested() && !opModeIsActive()) {

        }
        waitForStart();
        if (isStopRequested()) return;

        drive.setPoseEstimate(BlueFarLeft.start());

        drive.followTrajectorySequence(BlueFarLeft);
    }

}
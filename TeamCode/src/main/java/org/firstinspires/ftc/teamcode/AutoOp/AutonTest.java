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

import org.firstinspires.ftc.robotcore.internal.camera.delegating.DelegatingCaptureSequence;
import org.firstinspires.ftc.teamcode.modules.ArmV2;
import org.firstinspires.ftc.teamcode.modules.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.modules.drive.SampleMecanumDrive;
//import org.firstinspires.ftc.teamcode.RobotCore;
import org.firstinspires.ftc.teamcode.modules.ArmV2;
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

        ArmV2 pixelArm = new ArmV2(hardwareMap, telemetry);

        pixelArm.transition(ArmV2.EVENT.CLAW_CLOSE);
        //pixelArm.transition(ArmV2.EVENT.AUTON_START);

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





        TrajectorySequence BlueFarLeft = drive.trajectorySequenceBuilder(new Pose2d(-34.92, 64.44, Math.toRadians(90.00)))
                .lineToSplineHeading(new Pose2d(-35.07, 30.92, Math.toRadians(180)))
                .lineToSplineHeading(new Pose2d(-20.09, 31.07, Math.toRadians(180.00)))
                .lineToConstantHeading(new Vector2d(47.98, 31.96))
                .build();

        TrajectorySequence BlueFarRight = drive.trajectorySequenceBuilder(new Pose2d(-34.92, 62.21, Math.toRadians(-83.66)))
                .splineTo(new Vector2d(-31.37, 32.11), Math.toRadians(-1.94))
                .splineTo(new Vector2d(-11.79, 28.25), Math.toRadians(-87.74))
                .splineTo(new Vector2d(10.16, -3.63), Math.toRadians(22.50))
                .splineTo(new Vector2d(47.83, 34.92), Math.toRadians(0.00))
                .build();
        TrajectorySequence BlueFarCenter_0 = drive.trajectorySequenceBuilder(new Pose2d(-34.63, 63.25, Math.toRadians(90.00)))
                .splineToConstantHeading(new Vector2d(-34.63, 21.87), Math.toRadians(90.00))
                .lineToConstantHeading(new Vector2d(-34.63, 5.26))
                .lineToSplineHeading(new Pose2d(-0.67, 3.49, Math.toRadians(167.91)))
                .splineToSplineHeading(new Pose2d(51.09, 31.96, Math.toRadians(-4.09)), Math.toRadians(184.73))
                .build();

        TrajectorySequence newBlueFarCenter = drive.trajectorySequenceBuilder(new Pose2d(-38.19, 56.41, Math.toRadians(90.00)))
                //.splineTo(new Vector2d(-35.87, 27.88), Math.toRadians(89.71))
                .lineTo(new Vector2d(-36.04, 45.42))
                .splineTo(new Vector2d(-55.15, 40.91), Math.toRadians(267.61))
                .splineTo(new Vector2d(-56.71, 23.19), Math.toRadians(269.38))
                .splineTo(new Vector2d(0.26, 6.69), Math.toRadians(-3.18))
                .splineTo(new Vector2d(33.09, 7.03), Math.toRadians(0.61))
                .splineTo(new Vector2d(33.09, 33.44), Math.toRadians(1.91))
                .splineTo(new Vector2d(48.20, 33.61), Math.toRadians(0.66))
                .build();

        TrajectorySequence BlueFarCenter_p1 = drive.trajectorySequenceBuilder(
                new Pose2d(-36.70, 68.74, Math.toRadians(90.00)))
                .lineTo(new Vector2d(-36.41, 16))
                .build();

        TrajectorySequence BlueFarCenter_p2 = drive.trajectorySequenceBuilder(BlueFarCenter_p1.end())
                .setReversed(true)
                .splineTo(new Vector2d(-18.61, 12.23), Math.toRadians(7.57))
                .splineTo(new Vector2d(30.48, 14.76), Math.toRadians(20.35))
                .splineTo(new Vector2d(35, 37.40), Math.toRadians(0))
                .build();

        TrajectorySequence moveForward = drive.trajectorySequenceBuilder(BlueFarCenter_p2.end())
                .back(12)
                .build();


        while(!isStopRequested() && !opModeIsActive()) {

        }
        waitForStart();
        if (isStopRequested()) return;

        pixelArm.transition(ArmV2.EVENT.DRIVE_WITH_PIXEL_POS);
        pixelArm.transition(ArmV2.EVENT.CLAW_CLOSE);
        drive.setPoseEstimate(BlueFarCenter_p1.start());
        drive.followTrajectorySequence(BlueFarCenter_p1);
        pixelArm.transition(ArmV2.EVENT.DROP_PURPLE);
        sleep(1000);
        pixelArm.transition(ArmV2.EVENT.DROP_RIGHT_PIXEL);
        sleep(500);
        pixelArm.transition(ArmV2.EVENT.DRIVE_WITH_PIXEL_POS);

        drive.setPoseEstimate(BlueFarCenter_p2.start());
        drive.followTrajectorySequence(BlueFarCenter_p2);

        //pixelArm.transition(ArmV2.EVENT.BACK_DROP_AUTON);
        //drive.setPoseEstimate(moveForward.start());
        //drive.followTrajectorySequence(moveForward);
        //sleep(2000);
        //pixelArm.transition(ArmV2.EVENT.DROP_LEFT_PIXEL);

        int level1 = 0;

        while (!isStopRequested()) {
            drive.update();
            if (level1 == 0){
                pixelArm.transition(ArmV2.EVENT.DROP_BACKDROP);
                drive.setPoseEstimate(moveForward.start());
                drive.followTrajectorySequence(moveForward);
                //sleep(2000);
                level1++;
            }
            if (pixelArm.armReached == false)
            {
                pixelArm.update();
            }
            else
            {
                sleep(2000);

                pixelArm.transition(ArmV2.EVENT.DROP_LEFT_PIXEL);
                sleep(2000);

                break;

            }
//            pixelArm.update();

            telemetry.addLine("DROP POS");
            telemetry.update();
        }
    }

}
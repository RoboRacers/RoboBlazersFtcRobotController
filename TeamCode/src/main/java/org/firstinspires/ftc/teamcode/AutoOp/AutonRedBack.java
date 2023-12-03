package org.firstinspires.ftc.teamcode.AutoOp;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.modules.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.modules.trajectorysequence.TrajectorySequence;

import java.util.Arrays;


@Config
@Autonomous(name = "Auton Red Back", group = "23692")
public class AutonRedBack extends LinearOpMode {

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
        TrajectorySequence RedBackLeft = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(0)))

                .forward(8)
                .strafeRight(7)
                .forward(24)
                .back(8)
                .turn(-90)
                .back(32)
                //.turn(180)
                .build();

        TrajectorySequence RedBackCenter = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                .forward(37)
                .back(12)
                .turn(90)
                .back(36)
                //.turn(180)
                .build();



        while(!isStopRequested() && !opModeIsActive()) {

        }
        waitForStart();
        if (isStopRequested()) return;

        drive.setPoseEstimate(RedBackLeft.start());

        drive.followTrajectorySequence(RedBackLeft);

        drive.setPoseEstimate(RedBackCenter.start());

        drive.followTrajectorySequence(RedBackCenter);
    }

}
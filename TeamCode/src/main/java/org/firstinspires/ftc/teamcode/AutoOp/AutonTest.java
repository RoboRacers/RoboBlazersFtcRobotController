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
        TrajectorySequence redCenter = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(0)))// 90 is facing red
                //.lineTo(new Vector2d(36, 0))
                //.lineTo(new Vector2d(-36.00, -20.00))
                //.lineTo(new Vector2d(-50.00, -37.00))
                //.lineTo(new Vector2d(-50, -7.00))
                //.turn(Math.toRadians(-90)) //change after tuning to -90
                //.lineTo(new Vector2d(10, -7))

                .forward(32)
                .back(5)
                .strafeLeft(15)
                .forward(28)
                .turn(Math.toRadians(-90))
                .strafeRight(4)
                .back(72)
                .strafeLeft(26)
                .back(30)
                // add arm
                .strafeLeft(20)
                .back(10)
                .build();

        TrajectorySequence blueCenter = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(0)))// 90 is facing red
                //.lineTo(new Vector2d(36, 0))
                //.lineTo(new Vector2d(-36.00, -20.00))
                //.lineTo(new Vector2d(-50.00, -37.00))
                //.lineTo(new Vector2d(-50, -7.00))
                //.turn(Math.toRadians(-90)) //change after tuning to -90
                //.lineTo(new Vector2d(10, -7))

                .forward(32)
                .back(5)
                .strafeRight(15)
                .forward(28)
                .turn(Math.toRadians(90))
                .strafeLeft(4)
                .back(72)
                .strafeRight(26)
                .back(30)
                // add arm
                .strafeRight(20)
                .back(10)
                .build();

        TrajectorySequence redRightSide = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(0)))

                .forward(25) //1
                .turn(Math.toRadians(90))
                .strafeLeft(7)
//                .back(10)
//                .strafeRight(11)
//                .forward(5)
//                .strafeLeft(5)
                .forward(8) //2 L
                .back(16) // 3
                .turn(Math.toRadians(-90))
                .forward(27)
                .turn(Math.toRadians(-90))
                .strafeRight(4)
                .back(72)
                .strafeLeft(29)
                .back(30)
                // add arm
                .strafeRight(30)
                .back(10)
//                .strafeRight(10)
//                .forward(31)
//                .turn(Math.toRadians(90))
//                .strafeLeft(2)
//                .back(72)
//                .strafeRight(26)
//                .back(32)
                .build();



        TrajectorySequence blueLeftSide = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(0)))

                .forward(25) //1
                .turn(Math.toRadians(-90))
                .strafeRight(7)
//                .back(10)
//                .strafeRight(11)
//                .forward(5)
//                .strafeLeft(5)
                .forward(8) //2 L
                .back(16) // 3
                .turn(Math.toRadians(90))
                .forward(27)
                .turn(Math.toRadians(90))
                .strafeLeft(4)
                .back(72)
                .strafeRight(29)
                .back(30)
                // add arm
                .strafeLeft(30)
                .back(10)
//                .strafeRight(10)
//                .forward(31)
//                .turn(Math.toRadians(90))
//                .strafeLeft(2)
//                .back(72)
//                .strafeRight(26)
//                .back(32)
                .build();

        TrajectorySequence blueRightSide = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(0)))

                .forward(25) //1
                .turn(Math.toRadians(90))
                .strafeLeft(6)
                //.forward(6) // drop point
                .back(3) // 3
                //.turn(Math.toRadians(-90))
                .strafeLeft(27)
                .back(55)
                .strafeRight(23)
                .back(30)
                // add arm
                .strafeLeft(20)
                .back(10)
                .build();






        TrajectorySequence blueBackCenter = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(0)))// 90 is facing red
                //.lineTo(new Vector2d(36, 0))
                //.lineTo(new Vector2d(-36.00, -20.00))
                //.lineTo(new Vector2d(-50.00, -37.00))
                //.lineTo(new Vector2d(-50, -7.00))
                //.turn(Math.toRadians(-90)) //change after tuning to -90
                //.lineTo(new Vector2d(10, -7))

                .forward(32)
                .back(5)
                .turn(Math.toRadians(90))
                .strafeLeft(4)
                .back(30)
                .strafeLeft(22)
                .back(10)

                .build();

        TrajectorySequence blueBackRightSide = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(0)))

                .forward(25) //1
                .turn(Math.toRadians(90))
                .strafeLeft(6)
                //.forward(6) // drop point
                .back(37) // 3
                //.turn(Math.toRadians(-90))
                .strafeLeft(24)
                .back(10)

                .build();


        TrajectorySequence blueBackLeftSide = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(0)))

                .forward(25) //1
                .turn(Math.toRadians(-90))
                .strafeRight(7)
//                .back(10)
//                .strafeRight(11)
//                .forward(5)
//                .strafeLeft(5)
                .forward(5)
                .back(8) // 3
                .turn(Math.toRadians(180))
                .strafeRight(2)
                .back(40)
                // add arm
                .strafeLeft(30)
                .back(10)
//                .strafeRight(10)
//                .forward(31)
//                .turn(Math.toRadians(90))
//                .strafeLeft(2)
//                .back(72)
//                .strafeRight(26)
//                .back(32)
                .build();

        while(!isStopRequested() && !opModeIsActive()) {

        }
        waitForStart();
        if (isStopRequested()) return;

        drive.setPoseEstimate(blueBackLeftSide.start());

        drive.followTrajectorySequence(blueBackLeftSide);
    }

}
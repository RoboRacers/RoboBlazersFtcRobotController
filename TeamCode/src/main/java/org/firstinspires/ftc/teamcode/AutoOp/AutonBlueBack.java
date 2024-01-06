package org.firstinspires.ftc.teamcode.AutoOp;

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

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.modules.Arm;
import org.firstinspires.ftc.teamcode.modules.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.modules.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.Arrays;


@Config
@Autonomous(name = "Auton Blue Back", group = "23692")
public class AutonBlueBack extends LinearOpMode {

    boolean finished = false;
    public SampleMecanumDrive drive;

    String direction;

    private OpenCvWebcam camera;
    private TeamPropDetectionPipeline teamPropDetectionPipeline;

    @Override
    public void runOpMode() {
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Arm pixelArm = new Arm(hardwareMap, telemetry);
        pixelArm.resetEncoder();
        //pixelArm.startPosInAuton(-200);

        pixelArm.clawOpen();

        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        WebcamName mycam = hardwareMap.get(WebcamName.class, "Webcam 1");

        camera = OpenCvCameraFactory.getInstance().createWebcam(mycam, cameraMonitorViewId);

        teamPropDetectionPipeline = new TeamPropDetectionPipeline(camera, telemetry);

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

        TrajectorySequence BlueCloseCenter = drive.trajectorySequenceBuilder(new Pose2d(12.53, 59.69, Math.toRadians(90.00)))
                .lineTo(new Vector2d(10.46, 9.57))
                .setReversed(true)
                .splineTo(new Vector2d(36.11, 8.38), Math.toRadians(0.00))
                .lineTo(new Vector2d(41, 30.33))
                .build();
        TrajectorySequence BlueCloseLeft = drive.trajectorySequenceBuilder(new Pose2d(12.09, 59.54, Math.toRadians(90.00)))
                .lineTo(new Vector2d(11.05, 30.33))
                .setReversed(true)
                .splineTo(new Vector2d(43.08, 36.26), Math.toRadians(0.00))
                .build();
        TrajectorySequence BlueCloseRight = drive.trajectorySequenceBuilder(new Pose2d(11.49, 59.25, Math.toRadians(90.00)))
                .lineTo(new Vector2d(11.64, 44.42))
                .setReversed(true)
                .splineTo(new Vector2d(11.64, 32.85), Math.toRadians(0.00))
                .lineTo(new Vector2d(5.86, 33.29))
                .setReversed(false)
                .lineTo(new Vector2d(40.56, 21.43))
                .setReversed(true)
                .build();


        while (opModeInInit()) {

            direction = teamPropDetectionPipeline.getDirection();


            while (!isStopRequested()) {


                if (direction == "center") {
                    drive.setPoseEstimate(BlueCloseCenter.start());
                    drive.followTrajectorySequence(BlueCloseCenter);
                    break;
                }
                else if (direction == "left") {
                    drive.setPoseEstimate(BlueCloseLeft.start());
                    drive.followTrajectorySequence(BlueCloseLeft);
                    break;
                }
                else if (direction == "right") {
                    drive.setPoseEstimate(BlueCloseRight.start());
                    drive.followTrajectorySequence(BlueCloseRight);
                    break;
                }
                camera.closeCameraDevice();
                //return;
            }

        }
    }
}
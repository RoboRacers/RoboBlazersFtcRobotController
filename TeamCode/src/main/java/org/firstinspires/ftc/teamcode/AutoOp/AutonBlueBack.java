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

        //RobotCore robot = new RobotCore(hardwareMap);
        TrajectorySequence BlueBackLeft = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(0)))

                .forward(8)
                .strafeLeft(7)
                .forward(24)
                .back(8)
                .turn(90)
                .back(32)
                //.turn(180)
                .build();

        TrajectorySequence BlueBackCenter = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                .forward(37)
                .back(12)
                .turn(90)
                .back(36)
                //.turn(180)
                .build();


        while (opModeInInit()) {

            direction = teamPropDetectionPipeline.getDirection();


            while (!isStopRequested()) {


                if (direction == "center") {
                    drive.setPoseEstimate(BlueBackCenter.start());
                    drive.followTrajectorySequence(BlueBackCenter);
                    break;
                }
                else if (direction == "left") {
                    drive.setPoseEstimate(BlueBackLeft.start());
                    drive.followTrajectorySequence(BlueBackLeft);
                    break;
                }
                camera.closeCameraDevice();
                //return;
            }

        }
    }
}
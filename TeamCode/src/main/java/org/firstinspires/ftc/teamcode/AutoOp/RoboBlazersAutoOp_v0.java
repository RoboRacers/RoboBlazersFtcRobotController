package org.firstinspires.ftc.teamcode.AutoOp;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.modules.Arm;
import org.firstinspires.ftc.teamcode.modules.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.modules.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name = "Auton Test RoboBlazers")
public class RoboBlazersAutoOp_v0 extends LinearOpMode {
        private OpenCvWebcam camera;
        private TeamPropDetectionPipeline teamPropDetectionPipeline;

        int leftCount;
        int rightCount;
        int centerCount;
        String direction;
        long maxTimer = 2000;
        long startTime = System.currentTimeMillis();



    @Override
        public void runOpMode () throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Arm pixelArm = new Arm(hardwareMap, telemetry);
        pixelArm.resetEncoder();
        //pixelArm.startPosInAuton(-200);

        pixelArm.clawOpen();

        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                    .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        WebcamName mycam = hardwareMap.get(WebcamName.class, "Webcam 1");

        camera = OpenCvCameraFactory.getInstance().createWebcam(mycam, cameraMonitorViewId);

        teamPropDetectionPipeline = new TeamPropDetectionPipeline(camera, telemetry);

        Trajectory centerPos = drive.trajectoryBuilder(new Pose2d())
                .strafeLeft(4)
                .build();

        Trajectory trajectory1 =  drive.trajectoryBuilder(centerPos.end())
                .forward(29)
                .build();

        Trajectory back = drive.trajectoryBuilder(trajectory1.end())
                .back(2)
                .build();

        Trajectory goLittleForward = drive.trajectoryBuilder(centerPos.end())
            .forward(10)
            .build();

        Trajectory goToBackdrop = drive.trajectoryBuilder(goLittleForward.end())
                .strafeRight(30)
                .build();

        Trajectory goLittleRight = drive.trajectoryBuilder(trajectory1.end())
                .strafeLeft(4)
                .build();

        TrajectorySequence untitled0 = drive.trajectorySequenceBuilder(new Pose2d(-36.09, -67.99, Math.toRadians(90.55)))
                .splineTo(new Vector2d(-36.78, -33.82), Math.toRadians(90.67))
                .splineTo(new Vector2d(-41.14, -32.60), Math.toRadians(214.38))
                .splineTo(new Vector2d(-43.58, -37.48), Math.toRadians(172.06))
                .splineTo(new Vector2d(-47.07, -32.43), Math.toRadians(168.69))
                .splineTo(new Vector2d(-50.73, -35.91), Math.toRadians(105.95))
                .splineTo(new Vector2d(-52.30, -15.69), Math.toRadians(94.44))
                .splineTo(new Vector2d(-52.30, -9.59), Math.toRadians(90.00))
                .splineTo(new Vector2d(-27.54, -2.27), Math.toRadians(-1.32))
                .splineTo(new Vector2d(4.18, -3.49), Math.toRadians(0.00))
                .splineTo(new Vector2d(24.93, -0.87), Math.toRadians(5.49))
                .splineTo(new Vector2d(32.43, -22.31), Math.toRadians(261.25))
                .splineTo(new Vector2d(33.82, -48.12), Math.toRadians(270.00))
                .splineTo(new Vector2d(36.96, -52.13), Math.toRadians(40.82))
                .splineTo(new Vector2d(43.41, -58.75), Math.toRadians(-0.82))
                .splineTo(new Vector2d(62.06, -62.76), Math.toRadians(-15.95))
                .build();





        while (opModeInInit()) {

            direction = teamPropDetectionPipeline.getDirection();

            if (direction == "center") {
                centerCount++;
            } else if (direction == "left") {
                leftCount++;
            } else if (direction == "right") {
                rightCount++;
            }

//            if (leftCount > centerCount && leftCount > rightCount){
//                direction = "left";
//            } else if (centerCount > leftCount && centerCount > rightCount) {
//                direction = "center";
//            } else if (rightCount > leftCount & rightCount > centerCount) {
//                direction = "right";
//            }

            String time = teamPropDetectionPipeline.getElapsedTime();
            String endTime = teamPropDetectionPipeline.getEndTime();
            telemetry.addData("time: ", time);
            telemetry.addData("endTIme: ", endTime);
            telemetry.addData("direction:", direction);
            telemetry.update();
        }
        waitForStart();

            while (!isStopRequested()) {


                if(direction=="center"){
                    drive.setPoseEstimate(untitled0.start());
                    drive.followTrajectorySequence(untitled0);
                    break;
                }
                camera.closeCameraDevice();
            //return;
            }
        }
    }
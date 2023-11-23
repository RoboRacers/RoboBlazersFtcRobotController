package org.firstinspires.ftc.teamcode.AutoOp;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.modules.Arm;
import org.firstinspires.ftc.teamcode.modules.drive.SampleMecanumDrive;
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
        long maxTimer = 5000;
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
                    .strafeRight(4)
                    .build();

            Trajectory trajectory1 =  drive.trajectoryBuilder(centerPos.end())
                    .forward(29)
                    .build();

            Trajectory back = drive.trajectoryBuilder(trajectory1.end())
                    .back(2)
                    .build();

            Trajectory goToBackdrop = drive.trajectoryBuilder(back.end())
                    .strafeRight(30)
                    .build();

            while (System.currentTimeMillis() - startTime < maxTimer) {

                direction = teamPropDetectionPipeline.getDirection();

                if (direction == "center") {
                    centerCount++;
                } else if (direction == "left") {
                    leftCount++;
                } else if (direction == "right") {
                    rightCount++;
                }

                if (leftCount > centerCount && leftCount > rightCount){
                    direction = "left";
                } else if (centerCount > leftCount && centerCount > rightCount) {
                    direction = "center";
                } else if (rightCount > leftCount & rightCount > centerCount) {
                    direction = "right";
                }
                telemetry.addData("direction:", direction);
                telemetry.update();
            }
            waitForStart();

            while (!isStopRequested()) {


                if(direction=="center"){
                    sleep(2000);
                    pixelArm.dropInAuton();
                    sleep(1000);
                    drive.followTrajectory(centerPos);
                    Pose2d poseEstimate = drive.getPoseEstimate();
                    sleep(2000);
                    drive.followTrajectory(trajectory1);
                    pixelArm.dropOnePixel();
                    sleep(1000);
                    drive.followTrajectory(back);
                    sleep(2000);
                    drive.followTrajectory(goToBackdrop);
                    break;
                }
                return;
            }

            camera.closeCameraDevice();
        }
    }
package org.firstinspires.ftc.teamcode.AutoOp;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.modules.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.modules.drive.SampleMecanumDrive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.modules.Arm;

@Autonomous
public class RoboBlazersAutoOp_v0 extends LinearOpMode {
        private OpenCvWebcam camera;
        private TeamPropDetectionPipeline teamPropDetectionPipeline;

        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);



    @Override
        public void runOpMode () throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Arm pixelArm = new Arm(hardwareMap, telemetry);

        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                    .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

            WebcamName mycam = hardwareMap.get(WebcamName.class, "TPcam");

            camera = OpenCvCameraFactory.getInstance().createWebcam(mycam, cameraMonitorViewId);

            teamPropDetectionPipeline = new TeamPropDetectionPipeline(camera, telemetry);



            Trajectory centerPos = drive.trajectoryBuilder(new Pose2d())
                    .forward(32)
                    .build();

            Trajectory trajectory1 =  drive.trajectoryBuilder(centerPos.end())
                    .back(4)
                    .build();


            waitForStart();

            while (!isStopRequested()) {
                pixelArm.startPosInAuton();


                String direction = teamPropDetectionPipeline.getDirection();

                telemetry.addData("Direction", direction);
                telemetry.update();


                if(direction=="center"){
                    sleep(2000);
                    pixelArm.dropInAuton();
                    sleep(1000);
                    drive.followTrajectory(centerPos);
                    Pose2d poseEstimate = drive.getPoseEstimate();
                    sleep(2000);
                    pixelArm.clawOpen();

                }
            }

            camera.closeCameraDevice();
        }
    }
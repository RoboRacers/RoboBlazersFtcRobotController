package org.firstinspires.ftc.teamcode.AutoOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;

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

@TeleOp(name="Pixel allign OpMode", group="Linear Opmode")
public class PixelAllignmentTest extends LinearOpMode {

    public SampleMecanumDrive drive;


    @Override
    public void runOpMode() {

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        OpenCvWebcam camera;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().
                getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.
                get(WebcamName.class, "Webcam 2"), cameraMonitorViewId);

        PixelDetection myPixelDetection = new PixelDetection(camera, telemetry);

        double pixelCenterX = 0.0;
        double pixelCenterY = 0.0;

        TrajectorySequence right = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(0)))// 90 is facing red
                .strafeRight(1)
                .build();

        TrajectorySequence left = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(0)))// 90 is facing red
                .strafeLeft(1)
                .build();

        TrajectorySequence forward = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(0)))// 90 is facing red
                .forward(1)
                .build();

        TrajectorySequence back = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(0)))// 90 is facing red
                .back(1)
                .build();

        waitForStart();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            sleep(300);
            pixelCenterX = myPixelDetection.getCenterX();
            pixelCenterY = myPixelDetection.getCenterY();

            int DisplayCenterX = 320;
            int DisplayCenterY = 240;

            String directionX = "NAX";
            String directionY = "NAY";

            if (pixelCenterX > 0 && pixelCenterX < DisplayCenterX-150) {
                directionX = "left";
            } else if (pixelCenterX > DisplayCenterX+150 && pixelCenterX < DisplayCenterX*2) {
                directionX = "right";
            } else {
                directionX = "allign_x";
            }


            if (pixelCenterY > 0 && pixelCenterY < DisplayCenterY-150) {
                directionY = "up";
            } else if (pixelCenterY > DisplayCenterY+150 && pixelCenterY < DisplayCenterY*2) {
                directionY = "right";
            } else {
                directionY = "allign_y";
            }

            telemetry.addData(directionX, directionY);

            telemetry.addData("Pixel Center X = ", pixelCenterX);
            telemetry.addData("Pixel Center Y = ", pixelCenterY);
            telemetry.update();

            if (directionX == "right"){
                drive.setPoseEstimate(right.start());

                drive.followTrajectorySequence(right);
            }

            if (directionX == "left"){
                drive.setPoseEstimate(left.start());

                drive.followTrajectorySequence(left);
            }

            if (directionY == "down"){
                drive.setPoseEstimate(back.start());

                drive.followTrajectorySequence(back);
            }

            if (directionY == "up"){
                drive.setPoseEstimate(forward.start());

                drive.followTrajectorySequence(forward);
            }


        }
    }
}

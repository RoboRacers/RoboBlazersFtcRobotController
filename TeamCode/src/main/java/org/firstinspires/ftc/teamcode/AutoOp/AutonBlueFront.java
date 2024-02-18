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

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.modules.ArmV2;
import org.firstinspires.ftc.teamcode.modules.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.modules.drive.SampleMecanumDrive;
//import org.firstinspires.ftc.teamcode.RobotCore;
import org.firstinspires.ftc.teamcode.modules.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.Arrays;


@Config
@Autonomous(name = "Robotblazers BLUE FRONT", group = "23692")
public class AutonBlueFront extends LinearOpMode {

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

        ArmV2 pixelArm = new ArmV2(hardwareMap, telemetry);
        //pixelArm.startPosInAuton(-200);

        pixelArm.transition(ArmV2.EVENT.CLAW_CLOSE);
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

        TrajectorySequence BlueFarCenter_p1 = drive.trajectorySequenceBuilder(
                        new Pose2d(-36.70, 68.74, Math.toRadians(90.00)))
                .lineTo(new Vector2d(-36.41, 19.06))
                .build();

        TrajectorySequence BlueFarCenter_p2 = drive.trajectorySequenceBuilder(BlueFarCenter_p1.end())
//                .setReversed(true)
//                .splineTo(new Vector2d(-18.61, 12.23), Math.toRadians(7.57))
//                .splineTo(new Vector2d(30.48, 14.76), Math.toRadians(20.35))
//                .splineTo(new Vector2d(32, 29), Math.toRadians(6.71))
//                .build();
                .lineTo(new Vector2d(-36.53, 14.58))
                .setReversed(true)
                .splineTo(new Vector2d(-0.70, 10.36), Math.toRadians(-1.12))
                .setReversed(false)
                .lineTo(new Vector2d(38.11, 11.77))
                .setReversed(true)
                .lineTo(new Vector2d(32, 33))
                .build();

        TrajectorySequence moveForward = drive.trajectorySequenceBuilder(new Pose2d())
                .back(12)
                .build();

        TrajectorySequence BlueFarLeft_p1 = drive.trajectorySequenceBuilder(
                new Pose2d(-38.48, 61.92, Math.toRadians(90.00)))
//                .lineTo(new Vector2d(-37.05, 49.52))
//                .setReversed(true)
//                .splineTo(new Vector2d(-43.90, 37.23), Math.toRadians(180.00))
//                .lineTo(new Vector2d(-32, 30))
//                .setReversed(false)
//                .lineTo(new Vector2d(-20, 32))
//                .lineTo(new Vector2d(-33, 32))
//
//                .build();
                .lineTo(new Vector2d(-37.40, 44.96))
                .setReversed(true)
                .splineTo(new Vector2d(-42.67, 36.18), Math.toRadians(180.00))
                .lineTo(new Vector2d(-17.21, 30))
                .setReversed(false)
                .lineTo(new Vector2d(-33.72, 30))
                .build();

        TrajectorySequence BlueFarLeft_p2 = drive.trajectorySequenceBuilder(BlueFarLeft_p1.end())
                .lineTo(new Vector2d(-37.76, 0))
                .setReversed(true)
                .splineTo(new Vector2d(-38.28, 0), Math.toRadians(0.00))
                .setReversed(false)
                .lineTo(new Vector2d(30.56, 0))
                .lineTo(new Vector2d(32, 31))
                .build();
        TrajectorySequence BlueFarRight_p1 = drive.trajectorySequenceBuilder(new Pose2d(-37.15, 59.99, Math.toRadians(90.00)))
                .lineTo(new Vector2d(-48.72, 49.01))
                .setReversed(true)
                .lineTo(new Vector2d(-44, 10))
                .build();
        TrajectorySequence BlueFarRight_p2 = drive.trajectorySequenceBuilder(BlueFarRight_p1.end())
                .setReversed(true)
                .splineTo(new Vector2d(39.23, 10.75), Math.toRadians(0.00))
                .lineTo(new Vector2d(32, 18))
                .build();


        while (opModeInInit()) {

            direction = teamPropDetectionPipeline.getDirection();
            telemetry.addData("DIRECTION: ", direction);
            telemetry.update();
            //direction = "left";

        }
        camera.closeCameraDevice();


            waitForStart();
            if (isStopRequested()) {
                //camera.closeCameraDevice();
                return;
            }
            pixelArm.transition(ArmV2.EVENT.DRIVE_WITH_PIXEL_POS);
            pixelArm.transition(ArmV2.EVENT.CLAW_CLOSE);

            sleep(8000);


        if(direction == "left") {
                drive.setPoseEstimate(BlueFarLeft_p1.start());
                drive.followTrajectorySequence(BlueFarLeft_p1);
            } else if(direction == "right"){
                drive.setPoseEstimate(BlueFarRight_p1.start());
                drive.followTrajectorySequence(BlueFarRight_p1);
            }else{
                drive.setPoseEstimate(BlueFarCenter_p1.start());
                drive.followTrajectorySequence(BlueFarCenter_p1);
            }

            pixelArm.transition(ArmV2.EVENT.DROP_PURPLE);
            sleep(1000);
            pixelArm.transition(ArmV2.EVENT.DROP_LEFT_PIXEL);
            sleep(500);
            pixelArm.transition(ArmV2.EVENT.AUTON_POS);

            if(direction == "left"){
                drive.setPoseEstimate(BlueFarLeft_p2.start());
                drive.followTrajectorySequence(BlueFarLeft_p2);
            }
            else if (direction == "right") {
                drive.setPoseEstimate(BlueFarRight_p2.start());
                drive.followTrajectorySequence(BlueFarRight_p2);
            }else{
                drive.setPoseEstimate(BlueFarCenter_p2.start());
                drive.followTrajectorySequence(BlueFarCenter_p2);
            }

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

                    pixelArm.transition(ArmV2.EVENT.DROP_RIGHT_PIXEL);
                    sleep(2000);

                    break;

                }
            pixelArm.update();

                telemetry.addLine("DROP POS");
                //telemetry.update();
            }


    }
}
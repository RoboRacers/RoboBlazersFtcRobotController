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
@Autonomous(name = "Robotblazers RED BACK", group = "23692")
public class AutonRedBack extends LinearOpMode {

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
        6. +y towards Red
         */

        TrajectorySequence RedCloseCenter_p1 = drive.trajectorySequenceBuilder(new Pose2d(12.53, -59.69, Math.toRadians(-90.00)))
                .lineTo(new Vector2d(10.46, -9.57))
                .build();

        TrajectorySequence RedCloseCenter_p2 = drive.trajectorySequenceBuilder(RedCloseCenter_p1.end())
                .setReversed(true)
                .splineTo(new Vector2d(36.11, -8.38), Math.toRadians(-0.00))
                .lineTo(new Vector2d(41, -30.33))
                .build();

        TrajectorySequence moveForward = drive.trajectorySequenceBuilder(new Pose2d())
                .back(12)
                .build();

        TrajectorySequence RedCloseLeft_p1 = drive.trajectorySequenceBuilder(new Pose2d(12.09, -59.54, Math.toRadians(-90.00)))
                .lineTo(new Vector2d(11.05, -30.33))
                .build();
        TrajectorySequence RedCloseLeft_p2 = drive.trajectorySequenceBuilder(RedCloseLeft_p1.end())
                .setReversed(true)
                .splineTo(new Vector2d(43.08, -36.26), Math.toRadians(-0.00))
                .build();
        TrajectorySequence RedCloseRight_p1 = drive.trajectorySequenceBuilder(new Pose2d(11.49, -59.25, Math.toRadians(-90.00)))
                .lineTo(new Vector2d(11.64, -44.42))
                .build();
        TrajectorySequence RedCloseRight_p2 = drive.trajectorySequenceBuilder(RedCloseRight_p1.end())
                .setReversed(true)
                .splineTo(new Vector2d(11.64, -32.85), Math.toRadians(-0.00))
                .lineTo(new Vector2d(5.86, -33.29))
                .setReversed(false)
                .lineTo(new Vector2d(40.56, -21.43))
                .setReversed(true)
                .build();



        while (opModeInInit()) {

            direction = teamPropDetectionPipeline.getDirection();


            while(!isStopRequested() && !opModeIsActive()) {

            }
            waitForStart();
            if (isStopRequested()) return;

            pixelArm.transition(ArmV2.EVENT.DRIVE_WITH_PIXEL_POS);
            pixelArm.transition(ArmV2.EVENT.CLAW_CLOSE);

            if(direction == "left" ) {
                drive.setPoseEstimate(RedCloseLeft_p1.start());
                drive.followTrajectorySequence(RedCloseLeft_p1);
            } else if(direction == "right"){
                drive.setPoseEstimate(RedCloseRight_p1.start());
                drive.followTrajectorySequence(RedCloseRight_p1);
            }else{
                drive.setPoseEstimate(RedCloseCenter_p1.start());
                drive.followTrajectorySequence(RedCloseCenter_p1);
            }

            pixelArm.transition(ArmV2.EVENT.DROP_PURPLE);
            sleep(1000);
            pixelArm.transition(ArmV2.EVENT.DROP_RIGHT_PIXEL);
            sleep(500);
            pixelArm.transition(ArmV2.EVENT.DRIVE_WITH_PIXEL_POS);

            if(direction == "left"){
                drive.setPoseEstimate(RedCloseLeft_p2.start());
                drive.followTrajectorySequence(RedCloseLeft_p2);
            }
            else if (direction == "right") {
                drive.setPoseEstimate(RedCloseRight_p2.start());
                drive.followTrajectorySequence(RedCloseRight_p2);
            }else{
                drive.setPoseEstimate(RedCloseCenter_p2.start());
                drive.followTrajectorySequence(RedCloseCenter_p2);
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
}
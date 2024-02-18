package org.firstinspires.ftc.teamcode.AutoOp;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.modules.Arm;
import org.firstinspires.ftc.teamcode.modules.ArmV2;
import org.firstinspires.ftc.teamcode.modules.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.modules.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name = "Auton Test RoboBlazers V1")
public class RoboBlazersAutoOp_v0 extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        ArmV2 pixelArm = new ArmV2(hardwareMap, telemetry);


        TrajectorySequence starter = drive.trajectorySequenceBuilder(new Pose2d(44.67, -37.31, Math.toRadians(180.00)))
                .splineTo(new Vector2d(24.18, -59.56), Math.toRadians(180.00))
                .splineTo(new Vector2d(-44.15, -59.74), Math.toRadians(180.00))
                .splineTo(new Vector2d(-53.43, -48.00), Math.toRadians(123.27))
                .build();

        TrajectorySequence moveForward = drive.trajectorySequenceBuilder(new Pose2d())
                .turn(Math.toRadians(-10))

                .forward(10)

                .build();
        TrajectorySequence back = drive.trajectorySequenceBuilder(new Pose2d())
                .back(7)
                .build();

        TrajectorySequence backDrop = drive.trajectorySequenceBuilder(starter.end())
//                .lineTo(new Vector2d(-53.08, -50.10))
//                .setReversed(true)
//                .splineTo(new Vector2d(-41.17, -54.83), Math.toRadians(1.62))
//                .splineTo(new Vector2d(-20.85, -54.48), Math.toRadians(0.00))
//                .lineTo(new Vector2d(31.18, -52.55))
//                .splineTo(new Vector2d(48.00, -31.18), Math.toRadians(0.00))
//                .build();

                .lineTo(new Vector2d(-54.31, -54.13))
                .splineTo(new Vector2d(-57.28, -36.44), Math.toRadians(90.00))
                .splineTo(new Vector2d(-49.75, -11.74), Math.toRadians(0.00))
                .setReversed(true)
                .lineTo(new Vector2d(35.21, -11.91))
                .setReversed(false)
                .splineTo(new Vector2d(41.52, -37.49), Math.toRadians(0.00))
                .setReversed(true)
                .build();



        while (opModeInInit()) {
        }
        waitForStart();
        if (isStopRequested()) {
            telemetry.addLine("in stop");
            telemetry.update();
            return;
        }


        pixelArm.transition(ArmV2.EVENT.DRIVE_WITH_PIXEL_POS);
        pixelArm.transition(ArmV2.EVENT.CLAW_CLOSE);

        telemetry.addLine("DONE INIT,STARTING");

        drive.setPoseEstimate(starter.start());
        drive.followTrajectorySequence(starter);


        int level1 = 0;

        while (!isStopRequested()) {
            drive.update();
            if (level1 == 0) {
                pixelArm.transition(ArmV2.EVENT.PIXEL_PICK_UP);

                //sleep(2000);
                level1++;
            }
            if (pixelArm.armReached == false) {
                pixelArm.update();
            } else {
                //sleep(3000);
                //pixelArm.transition(ArmV2.EVENT.PIXEL_PICK_UP);
                sleep(1000);

                drive.setPoseEstimate(moveForward.start());
                drive.followTrajectorySequence(moveForward);
                sleep(2000);

                pixelArm.transition(ArmV2.EVENT.CLAW_CLOSE);
                sleep(1000);
                pixelArm.update();
                pixelArm.transition(ArmV2.EVENT.ARM_STOP);
                pixelArm.update();
                drive.setPoseEstimate(back.start());
                drive.followTrajectorySequence(back);
                pixelArm.update();
                drive.setPoseEstimate(backDrop.start());
                drive.followTrajectorySequence(backDrop);



                break;

            }
//            pixelArm.update();

            telemetry.addLine("DROP POS");
            telemetry.update();

        }

    }
}

package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.modules.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.modules.trajectorysequence.TrajectorySequence;

import java.util.concurrent.TimeUnit;

@TeleOp(name = "Pixel Allign HuskyLens", group = "Sensor")
public class HuskyLensAllignWithStop extends LinearOpMode {

    private final int READ_PERIOD = 1;

    int pixelCenterX;
    int pixelCenterY;

    private HuskyLens huskyLens;

    public SampleMecanumDrive drive;



    private final int TAUGHT_OBJECT_ID = 1;

    @Override
    public void runOpMode()
    {
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        Deadline rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);

        rateLimit.expire();

        if (!huskyLens.knock()) {
            telemetry.addData(">>", "Problem communicating with " + huskyLens.getDeviceName());
        } else {
            telemetry.addData(">>", "Press start to continue");
        }


        huskyLens.selectAlgorithm(HuskyLens.Algorithm.OBJECT_TRACKING);

        telemetry.update();


        TrajectorySequence right = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(0)))// 90 is facing red
                .strafeRight(10)
                .build();

        TrajectorySequence left = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(0)))// 90 is facing red
                .strafeLeft(10)
                .build();

        TrajectorySequence forward = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(0)))// 90 is facing red
                .forward(10)
                .build();

        TrajectorySequence back = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(0)))// 90 is facing red
                .back(10)
                .build();

        TrajectorySequence stop = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(0)))// 90 is facing red
                .back(0.1)
                .build();

        waitForStart();


        while(opModeIsActive()) {
            if (!rateLimit.hasExpired()) {
                continue;
            }
            rateLimit.reset();

            HuskyLens.Block[] blocks = huskyLens.blocks();
            telemetry.addData("Block count", blocks.length);
            for (int i = 0; i < blocks.length; i++) {
                telemetry.addData("Block", blocks[i].toString());
                telemetry.addData("x: ", blocks[i].x);
                telemetry.addData("y: ", blocks[i].y);
                telemetry.addData("height: ", blocks[i].height);
                telemetry.addData("width: ", blocks[i].width);

                pixelCenterX = blocks[i].x + (blocks[i].width / 2);
                pixelCenterY = blocks[i].y + (blocks[i].height / 2);
            }


            telemetry.update();

            if (blocks.length != 0) {

                int DisplayCenterX = 320 / 2;
                int DisplayCenterY = 240 / 2;

                String directionX = "NAX";
                String directionY = "NAY";

                if (pixelCenterX > 0 && pixelCenterX < DisplayCenterX - 40) {
                    directionX = "left";
                } else if (pixelCenterX > DisplayCenterX + 40 && pixelCenterX < DisplayCenterX * 2) {
                    directionX = "right";
                } else if (pixelCenterX > DisplayCenterX - 40 && pixelCenterX < DisplayCenterX + 40) {
                    directionX = "allign_x";
                }


                if (pixelCenterY > 0 && pixelCenterY < DisplayCenterY - 40) {
                    directionY = "up";
                } else if (pixelCenterY > DisplayCenterY + 40 && pixelCenterY < DisplayCenterY * 2) {
                    directionY = "down";
                } else if (pixelCenterY > DisplayCenterY - 40 && pixelCenterY < DisplayCenterY + 40) {
                    directionY = "allign_y";
                }

                telemetry.addData(directionX, directionY);

                telemetry.addData("Pixel Center X = ", pixelCenterX);
                telemetry.addData("Pixel Center Y = ", pixelCenterY);
                telemetry.update();

                if (directionX == "right") {
                    drive.setPoseEstimate(right.start());

                    drive.followTrajectorySequenceAsync(right);

                    if (directionX == "allign_x") {
                        drive.breakFollowing();
                        drive.setDrivePower(new Pose2d());
                    }
                }

                if (directionX == "left") {
                    drive.setPoseEstimate(left.start());

                    drive.followTrajectorySequenceAsync(left);

                    if (directionX == "allign_x") {
                        drive.breakFollowing();
                        drive.setDrivePower(new Pose2d());

                    }
                }

//                if (directionY == "down") {
//                    drive.setPoseEstimate(back.start());
//
//                    drive.followTrajectorySequence(back);
//
//                    if (directionY == "allign_y") {
//                        drive.breakFollowing();
//                        drive.setDrivePower(new Pose2d());
//
//                    }
//                }
//
//                if (directionY == "up") {
//                    drive.setPoseEstimate(forward.start());
//
//                    drive.followTrajectorySequence(forward);
//
//                    if (directionY == "allign_y") {
//                        drive.breakFollowing();
//                        drive.setDrivePower(new Pose2d());
//
//                    }
//                }


            }
        }
    }
}

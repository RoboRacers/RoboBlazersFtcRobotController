package org.firstinspires.ftc.teamcode.AutoOp;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.modules.Arm;
import org.firstinspires.ftc.teamcode.modules.ArmV2;
import org.firstinspires.ftc.teamcode.modules.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.modules.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.Arrays;

@Autonomous(name = "Auton Test RoboBlazers V2-Starter Stack")
public class RedCloseStarterStackTest extends LinearOpMode {
    private OpenCvWebcam camera;
    private TeamPropDetectionPipeline teamPropDetectionPipeline;
    private String direction;

    @Override
    public void runOpMode() throws InterruptedException {



        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        ArmV2 pixelArm = new ArmV2(hardwareMap, telemetry);

        DcMotorEx armM = hardwareMap.get(DcMotorEx.class, "armMotor");

        ArmV2.Link1 links = pixelArm.new Link1(hardwareMap.get(Servo.class, "linkLeft"),
                hardwareMap.get(Servo.class, "linkRight"));

        ArmV2.Claw claw = pixelArm.new Claw(hardwareMap.get(Servo.class, "clawLeft"),
                hardwareMap.get(Servo.class, "clawRight"));

        Servo clawLeft = hardwareMap.get(Servo.class, "clawLeft");
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
        6. +y towards RED
         */

        TrajectorySequence REDCloseCenter_p1 = drive.trajectorySequenceBuilder(new Pose2d(12.53, -59.69, Math.toRadians(-90.00)))
                //.lineTo(new Vector2d(10.46, -9.57))
                //.build();
                .lineTo(new Vector2d(13.49, -45.72))
                .setReversed(true)
                .splineTo(new Vector2d(23.12, -16.47), Math.toRadians(-4.57))
                .build();


        TrajectorySequence REDCloseCenter_p2 = drive.trajectorySequenceBuilder(REDCloseCenter_p1.end())
//                .setReversed(true)
//                .splineTo(new Vector2d(36.11, -8.38), Math.toRadians(0.00))
//                .lineTo(new Vector2d(30, -29))
//                .build();
                .setReversed(true)
                .lineTo(new Vector2d(30, -28))
                .build();

        TrajectorySequence moveForward12 = drive.trajectorySequenceBuilder(new Pose2d())
                .back(8)
                .build();

        TrajectorySequence REDCloseLeft_p1 = drive.trajectorySequenceBuilder(new Pose2d(12.09, -59.54, Math.toRadians(-90.00)))
                .lineTo(new Vector2d(12.62, -43.97))
                .setReversed(true)
                .splineTo(new Vector2d(32.5, -28.78), Math.toRadians(-0.00))
                .build();
        TrajectorySequence REDCloseLeft_p2 = drive.trajectorySequenceBuilder(REDCloseLeft_p1.end())
                .setReversed(true)
                .lineTo(new Vector2d(30, -32))
                .build();
        TrajectorySequence REDCloseRight_p1 = drive.trajectorySequenceBuilder(new Pose2d(11.49, -59.25, Math.toRadians(-90.00)))
                .lineTo(new Vector2d(12.47, -41.62))
                .setReversed(true)
                .splineTo(new Vector2d(21.25, -28.98), Math.toRadians(-2.20))
                .lineTo(new Vector2d(7, -28.80))
                .setReversed(false)
                .build();

        TrajectorySequence REDCloseRight_p2 = drive.trajectorySequenceBuilder(REDCloseRight_p1.end())
                .lineTo(new Vector2d(15, -22))
                .setReversed(true)
                .build();


        TrajectorySequence starter = drive.trajectorySequenceBuilder(new Pose2d(43.27, -34.69, Math.toRadians(180.00)))
                .splineTo(new Vector2d(29.96, -61), Math.toRadians(181.47))
                .splineTo(new Vector2d(-52.91, -61), Math.toRadians(180.00))
                .splineTo(new Vector2d(-56.58, -43.97), Math.toRadians(135.00))
                .build();


        TrajectorySequence turnMoveForward = drive.trajectorySequenceBuilder(new Pose2d())
                .turn(Math.toRadians(-10))

                .forward(10)

                .build();
        TrajectorySequence back = drive.trajectorySequenceBuilder(new Pose2d())
                .back(7)
                .build();

        TrajectorySequence backDrop = drive.trajectorySequenceBuilder(starter.end())
                .lineTo(new Vector2d(-54.31, -54.13))
                .splineTo(new Vector2d(-57.28, -36.44), Math.toRadians(90.00))
                .splineTo(new Vector2d(-49.75, -11.74), Math.toRadians(0.00))
                .setReversed(true)
                .lineTo(new Vector2d(35.21, -11.91))
                .setReversed(false)
                .splineTo(new Vector2d(30, -37.49), Math.toRadians(180))
                .setReversed(true)
                .build();




        while (opModeInInit()) {
            direction = teamPropDetectionPipeline.getDirection();
            telemetry.addData("DIRECTION: ", direction);
            telemetry.update();
            //direction = "left";
        }
        waitForStart();
        if (isStopRequested()) {
            telemetry.addLine("in stop");
            telemetry.update();
            return;
        }


        //pixelArm.transition(ArmV2.EVENT.DRIVE_WITH_PIXEL_POS);
        //pixelArm.transition(ArmV2.EVENT.CLAW_CLOSE);
        links.moveLinks(1);
        claw.close();


        telemetry.addLine("DONE INIT,STARTING");


        if(direction == "right") {
            drive.setPoseEstimate(REDCloseLeft_p1.start());
            drive.followTrajectorySequence(REDCloseLeft_p1);
        } else if(direction == "left"){
            telemetry.addLine("Starting 1st part");
            drive.setPoseEstimate(REDCloseRight_p1.start());
            drive.followTrajectorySequence(REDCloseRight_p1);
        }else{
            drive.setPoseEstimate(REDCloseCenter_p1.start());
            drive.followTrajectorySequence(REDCloseCenter_p1);
        }

//        pixelArm.transition(ArmV2.EVENT.AUTON_POS);
//        sleep(1000);
//        pixelArm.transition(ArmV2.EVENT.DROP_LEFT_PIXEL);
//        sleep(500);
//        pixelArm.transition(ArmV2.EVENT.PID);

        links.moveLinks(0);
        pixelArm.update();
        sleep(700);
//        claw.openLeft();
        pixelArm.transition(ArmV2.EVENT.DROP_LEFT_PIXEL);
        //sleep(1000);
        //clawLeft.setPosition(0.5);
        links.moveLinks(1);

        if(direction == "right"){
            drive.setPoseEstimate(REDCloseLeft_p2.start());
            drive.followTrajectorySequence(REDCloseLeft_p2);
        }
        else if (direction == "left") {
            telemetry.addLine("Starting 2nd Part");

            drive.setPoseEstimate(REDCloseRight_p2.start());
            drive.followTrajectorySequence(REDCloseRight_p2);
        }else{
            drive.setPoseEstimate(REDCloseCenter_p2.start());
            drive.followTrajectorySequence(REDCloseCenter_p2);
        }


//        pixelArm.transition(ArmV2.EVENT.DRIVE_WITH_PIXEL_POS);
//        pixelArm.transition(ArmV2.EVENT.CLAW_CLOSE);

        telemetry.addLine("DONE INIT,STARTING");




        int level1 = 0;

        while (!isStopRequested()) {

            drive.update();
            if (level1 == 0){
                pixelArm.transition(ArmV2.EVENT.DROP_BACKDROP);
                pixelArm.update();
                drive.setPoseEstimate(moveForward12.start());
                drive.followTrajectorySequence(moveForward12);
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
                sleep(1000);

                pixelArm.transition(ArmV2.EVENT.DRIVE_WITH_PIXEL_POS);
//                armM.setPower(-0.5);
                pixelArm.update();
                sleep(1000);
                pixelArm.update();
                drive.setPoseEstimate(starter.start());
                drive.followTrajectorySequence(starter);
                armM.setPower(0);
                pixelArm.update();

                drive.setPoseEstimate(turnMoveForward.start());
                drive.followTrajectorySequence(turnMoveForward);
                sleep(1000);

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

                pixelArm.transition(ArmV2.EVENT.DROP_BACKDROP);
                pixelArm.update();
                sleep(2000);
                pixelArm.update();
                pixelArm.transition(ArmV2.EVENT.DROP_BOTH_PIXELS);

                break;

            }

//            drive.update();
//            if (level1 == 1) {
//                pixelArm.transition(ArmV2.EVENT.PIXEL_PICK_UP);
//
//                //sleep(2000);
//                level1++;
//            }
//            if (pixelArm.armReached == false) {
//                pixelArm.update();
//            } else {
//                //sleep(3000);
//                //pixelArm.transition(ArmV2.EVENT.PIXEL_PICK_UP);
//                sleep(1000);
//
//                drive.setPoseEstimate(turnMoveForward.start());
//                drive.followTrajectorySequence(turnMoveForward);
//                sleep(2000);
//
//                pixelArm.transition(ArmV2.EVENT.CLAW_CLOSE);
//                sleep(1000);
//                pixelArm.update();
//                pixelArm.transition(ArmV2.EVENT.ARM_STOP);
//                pixelArm.update();
//                drive.setPoseEstimate(back.start());
//                drive.followTrajectorySequence(back);
//                pixelArm.update();
//                drive.setPoseEstimate(backDrop.start());
//                drive.followTrajectorySequence(backDrop);
//
//
//
//                break;

            }
//            pixelArm.update();

            telemetry.addLine("DROP POS");
            telemetry.update();

        }

    }


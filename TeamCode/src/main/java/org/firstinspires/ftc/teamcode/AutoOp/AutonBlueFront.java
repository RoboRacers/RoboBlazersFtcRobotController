package org.firstinspires.ftc.teamcode.AutoOp;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.modules.Arm;
import org.firstinspires.ftc.teamcode.modules.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.modules.drive.SampleMecanumDrive;
//import org.firstinspires.ftc.teamcode.RobotCore;
import org.firstinspires.ftc.teamcode.modules.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.Arrays;


@Config
@Autonomous(name = "Robotblazers blue front", group = "23692")
public class AutonBlueFront extends LinearOpMode {

    boolean finished = false;
    public SampleMecanumDrive drive;

    String direction;

    RevTouchSensor touchSensor;

    static boolean isPressed;

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

        WebcamName mycam = hardwareMap.get(WebcamName.class, "TPcam");
        touchSensor = hardwareMap.get(RevTouchSensor.class, "touch");


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
        TrajectorySequence AutonBlueCenter = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                .forward(32)
                .back(5)
                .strafeRight(15)
                .forward(28)
                .turn(Math.toRadians(90))
                .strafeLeft(4)
                .back(72)
                .strafeRight(26)
                .back(30)
                // add arm
//                .strafeRight(20)
//                .back(10)
                .build();

        TrajectorySequence AutonBlueRight = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                .forward(25) //1
                .turn(Math.toRadians(90))
                .strafeLeft(6)
                .back(3) // 3
                .strafeLeft(27)
                .back(55)
                .strafeRight(23)
                .back(30)
                // add arm
//                .strafeLeft(20)
//                .back(10)
                .build();

        TrajectorySequence AutonBlueLeft = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                .forward(25) //1
                .turn(Math.toRadians(-90))
                .strafeRight(7)
                .forward(8) //2 L
                .back(16) // 3
                .turn(Math.toRadians(90))
                .forward(27)
                .turn(Math.toRadians(90))
                .strafeLeft(4)
                .back(72)
                .strafeRight(29)
                .back(30)
                // add arm

                .build();

        TrajectorySequence AutonPark = drive.trajectorySequenceBuilder(new Pose2d(0,0, Math.toRadians(0)))
//                .strafeLeft(25)
//                .back(13)
                .forward(1)
                .build();

        TrajectorySequence moveForward = drive.trajectorySequenceBuilder(new Pose2d(0,0, Math.toRadians(0)))
                .forward(6)
                .build();

        while (opModeInInit()) {

        }


        while (!isStopRequested()) {
            pixelArm.setLink(0);
            sleep(3000);

            for(int i =0; i<5; i++) {
                if (teamPropDetectionPipeline.teamPropDetectionPipeline != null) {
                    direction = teamPropDetectionPipeline.getDirection();
                    telemetry.addData("Direction: ", direction);
                    telemetry.update();
                    sleep(500);
                }
            }
            camera.closeCameraDevice();

            isPressed = touchSensor.isPressed();

            pixelArm.moveArmDown();
//            pixelArm.setLink(1);
            pixelArm.autonArmPos();
            pixelArm.setLink(1);


            if (isPressed){
                pixelArm.resetEncoder();
                pixelArm.autonArmPos();
                telemetry.addLine("TOUCH PRESSED");
                telemetry.update();
            }


            //sleep(2000);

            if (direction == "center") {
                pixelArm.autonArmPos();
                drive.setPoseEstimate(AutonBlueCenter.start());
                drive.followTrajectorySequence(AutonBlueCenter);
                pixelArm.armSetDropPos();
                sleep(2000);
                pixelArm.clawClose();
                sleep(2000);
                drive.setPoseEstimate(moveForward.start());
                drive.followTrajectorySequence(moveForward);
                pixelArm.armSetPickPos();
                sleep(1000);
                drive.setPoseEstimate(AutonPark.start());
                drive.followTrajectorySequence(AutonPark);
                break;
            }
            else if (direction == "right") {
                pixelArm.autonArmPos();

                drive.setPoseEstimate(AutonBlueRight.start());
                drive.followTrajectorySequence(AutonBlueRight);
                pixelArm.armSetDropPos();
                sleep(2000);
                pixelArm.clawClose();
                sleep(2000);
                drive.setPoseEstimate(moveForward.start());
                drive.followTrajectorySequence(moveForward);
                pixelArm.armSetPickPos();
                sleep(1000);
                drive.setPoseEstimate(AutonPark.start());
                drive.followTrajectorySequence(AutonPark);
                break;
            } else {
                pixelArm.autonArmPos();

                drive.setPoseEstimate(AutonBlueLeft.start());
                drive.followTrajectorySequence(AutonBlueLeft);
                pixelArm.armSetDropPos();
                sleep(2000);
                pixelArm.clawClose();
                sleep(2000);
                drive.setPoseEstimate(moveForward.start());
                drive.followTrajectorySequence(moveForward);
                pixelArm.armSetPickPos();
                sleep(1000);
                drive.setPoseEstimate(AutonPark.start());
                drive.followTrajectorySequence(AutonPark);
            }

            //return;
        }
    }

}
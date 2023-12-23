package org.firstinspires.ftc.teamcode.AutoOp;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
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
import org.firstinspires.ftc.teamcode.modules.Lift;
import org.firstinspires.ftc.teamcode.modules.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.modules.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.Arrays;


@Config
@Autonomous(name = "Auton Blue Back", group = "23692")
public class AutonBlueBack extends LinearOpMode {

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
        TrajectorySequence BlueBackRight = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(0)))

                .forward(25) //1
                .turn(Math.toRadians(90))
                .strafeLeft(6)
                //.forward(6) // drop point
                .back(27) // 3
                //.turn(Math.toRadians(-90))
//                .strafeLeft(24)
//                .back(10)
                .build();

        TrajectorySequence BlueBackLeft = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(0)))

                .forward(25) //1
                .turn(Math.toRadians(-90))
                .strafeRight(7)
                .forward(5)
                .back(8) // 3
                .turn(Math.toRadians(180))
                .strafeRight(2)
                .back(30)
                // add arm
//                .strafeLeft(30)
//                .back(10)
                .build();

        TrajectorySequence BlueBackCenter = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                .forward(32)
                .back(5)
                .turn(Math.toRadians(90))
                .strafeLeft(4)
                .back(22)
//                .strafeLeft(22)
//                .back(10)

                .build();


        TrajectorySequence AutonPark = drive.trajectorySequenceBuilder(new Pose2d(0,0, Math.toRadians(0)))
//                .strafeLeft(25)
//                .back(13)
                .forward(2)
                .build();

        TrajectorySequence moveForward = drive.trajectorySequenceBuilder(new Pose2d(0,0, Math.toRadians(0)))
                .forward(3)
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


            pixelArm.moveArmDown();
//            pixelArm.setLink(1);
            pixelArm.autonArmPos();
            pixelArm.setLink(1);


//            if (isPressed){
//                pixelArm.resetEncoder();
//                pixelArm.autonArmPos();
//                telemetry.addLine("TOUCH PRESSED");
//                telemetry.update();
//            }


            //sleep(2000);

            if (direction == "center") {
                pixelArm.autonArmPos();
                drive.setPoseEstimate(BlueBackCenter.start());
                drive.followTrajectorySequence(BlueBackCenter);
                pixelArm.armSetDropPos();
                sleep(2000);
                pixelArm.clawClose();
                sleep(2000);
                drive.setPoseEstimate(moveForward.start());
                drive.followTrajectorySequence(moveForward);
                drive.setPoseEstimate(AutonPark.start());
                drive.followTrajectorySequence(AutonPark);
                pixelArm.armSetPickPos();
                //sleep(1000);

                break;
            }
            else if (direction == "right") {
                pixelArm.autonArmPos();

                drive.setPoseEstimate(BlueBackRight.start());
                drive.followTrajectorySequence(BlueBackRight);
                pixelArm.armSetDropPos();
                sleep(2000);
                pixelArm.clawClose();
                sleep(2000);
                drive.setPoseEstimate(moveForward.start());
                drive.followTrajectorySequence(moveForward);
                drive.setPoseEstimate(AutonPark.start());
                drive.followTrajectorySequence(AutonPark);
                pixelArm.armSetPickPos();
                //sleep(1000);

                break;
            } else {
                pixelArm.autonArmPos();

                drive.setPoseEstimate(BlueBackLeft.start());
                drive.followTrajectorySequence(BlueBackLeft);
                pixelArm.armSetDropPos();
                sleep(2000);
                pixelArm.clawClose();
                sleep(2000);
                drive.setPoseEstimate(moveForward.start());
                drive.followTrajectorySequence(moveForward);
                sleep(1000);
                drive.setPoseEstimate(AutonPark.start());
                drive.followTrajectorySequence(AutonPark);
                pixelArm.armSetPickPos();
//                sleep(1000);

            }

            //return;
        }
    }

}
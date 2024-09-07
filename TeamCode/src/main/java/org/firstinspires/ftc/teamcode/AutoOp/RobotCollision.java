package org.firstinspires.ftc.teamcode.AutoOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import org.firstinspires.ftc.teamcode.modules.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.modules.trajectorysequence.TrajectorySequence;

@Autonomous(name = "Avoid Object and Move to Point with HuskyLens", group = "Auto")
public class RobotCollision extends LinearOpMode {

    private SampleMecanumDrive drive;
    private HuskyLens huskyLens;

    @Override
    public void runOpMode() {
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize HuskyLens
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");

        if (!huskyLens.knock()) {
            telemetry.addData(">>", "Problem communicating with " + huskyLens.getDeviceName());
        } else {
            telemetry.addData(">>", "Press start to continue");
        }

        // Select the appropriate algorithm for object detection
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);

        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        // Main loop: Detect object, avoid it, and then move to the final point
        while (opModeIsActive()) {
            HuskyLens.Block[] blocks = huskyLens.blocks();

            if (blocks.length > 0) {
                // Assume the first detected object is the target
                HuskyLens.Block targetBlock = blocks[0];

                // Calculate the object's position relative to the robot
                double objectX = targetBlock.x;
                double objectY = targetBlock.y;

                // Set the target point to move to after avoiding the object
                Vector2d targetPosition = new Vector2d(30, 30);  // Example coordinates for the final target

                // Determine a point to go around the object
                double avoidDistance = 15; // Distance to keep away from the object

                // Compute a new point to navigate around the object
                Vector2d objectPosition = new Vector2d(objectX, objectY);
                Vector2d avoidPoint = objectPosition.plus(new Vector2d(avoidDistance, avoidDistance)); // Adjust based on orientation

                // Get current robot pose
                Pose2d currentPose = drive.getPoseEstimate();

                // Create a trajectory to avoid the object and then go to the final point
                TrajectorySequence avoidAndMoveToPoint = drive.trajectorySequenceBuilder(currentPose)
                        .lineTo(avoidPoint)
                        .lineTo(targetPosition)
                        .build();

                // Follow the trajectory
                drive.followTrajectorySequence(avoidAndMoveToPoint);

                // Break after reaching the target point
                break;
            }

            // Update the drive and telemetry
            drive.update();
            telemetry.addData("Detected Blocks", blocks.length);
            telemetry.update();
        }

        // Stop HuskyLens after moving
        huskyLens.close();
    }
}

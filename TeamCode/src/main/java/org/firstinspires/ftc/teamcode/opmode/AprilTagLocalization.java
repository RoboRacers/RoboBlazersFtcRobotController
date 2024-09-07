package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "AprilTag Localization", group = "Concept")
public class AprilTagLocalization extends LinearOpMode {

    // Name of the webcam in the hardware map
    private static final String WEBCAM_NAME = "Webcam 1";

    // Size of the AprilTag (meters)
    private static final double TAG_SIZE = 0.166; // Example: 166mm or 0.166 meters

    // AprilTag processor and vision portal
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;

    @Override
    public void runOpMode() {

        // Initialize AprilTag detection
        initAprilTagDetection();

        // Telemetry to indicate initialization is complete
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                // Get the list of detected AprilTags
                List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();

                if (currentDetections.isEmpty()) {
                    telemetry.addLine("No tags detected");
                } else {
                    // Iterate through the detected tags
                    for (AprilTagDetection detection : currentDetections) {
                        // Known position and orientation of the detected AprilTag on the field
                        // These values should be set based on your field setup
                        double tagPosX = -30; // Example: X coordinate in meters
                        double tagPosY = 60; // Example: Y coordinate in meters
                        double tagYaw = 0; // Example: Yaw (orientation) in degrees

                        // Detected relative position and orientation from the robot's camera
                        double relativeX = detection.ftcPose.x; // in meters
                        double relativeY = detection.ftcPose.y; // in meters
                        double relativeYaw = detection.ftcPose.yaw; // in degrees
                        telemetry.addData("Camera Relative YAW", relativeYaw);

                        // Convert relative position to field coordinates
                        double cosTheta = Math.cos(Math.toRadians(tagYaw));
                        double sinTheta = Math.sin(Math.toRadians(tagYaw));

                        double robotPosX = tagPosX - (relativeX * cosTheta - relativeY * sinTheta);
                        double robotPosY = tagPosY - (relativeX * sinTheta + relativeY * cosTheta);

                        // The robot's orientation (yaw) relative to the field
                        double robotYaw = tagYaw + relativeYaw;

                        // Output robot's position and orientation on the field
                        telemetry.addData("Tag ID", detection.id);
                        telemetry.addData("Robot X on Field (m)", robotPosX);
                        telemetry.addData("Robot Y on Field (m)", robotPosY);
                        telemetry.addData("Robot Yaw on Field (deg)", robotYaw);
                    }
                }

                // Update telemetry
                telemetry.update();

                // Sleep to avoid high CPU usage
                sleep(20);
            }
        }

        // Close the vision portal
        visionPortal.close();
    }

    /**
     * Initialize the AprilTag detection.
     */
    private void initAprilTagDetection() {
        // Create the AprilTag processor with specified parameters
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawTagOutline(true)
                .build();

        // Create the vision portal with the webcam
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTagProcessor)
                .build();
    }

}

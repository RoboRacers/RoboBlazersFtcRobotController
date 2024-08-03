package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.modules.drive.SampleMecanumDrive;

@Config
@TeleOp(name = "PID to Point", group = "16481-Centerstage")
public class PIDPointImp extends LinearOpMode {

    private SampleMecanumDrive drive;

    public static double xkP = 0.1;
    public static double xkI = 0.01;
    public static double xkD = 0.01;
    public static double ykP = 0.1;
    public static double ykI = 0.01;
    public static double ykD = 0.01;
    public static double hkP = 0.1;
    public static double hkI = 0.01;
    public static double hkD = 0.01;

    public static double xTarget = 0;
    public static double yTarget = 0;
    public static double headingTarget = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d());

        CustomPID xPID = new CustomPID(xkP, xkI, xkD);
        CustomPID yPID = new CustomPID(ykP, ykI, ykD);
        CustomPID headingPID = new CustomPID(hkP, hkI, hkD);

        waitForStart();

        while (opModeIsActive()) {
            Pose2d currentPose = drive.getPoseEstimate();

            xPID.setTargetPosition(xTarget);
            yPID.setTargetPosition(yTarget);
            headingPID.setTargetPosition(Math.toRadians(headingTarget));

            xPID.setTunings(xkP, xkI, xkD);
            yPID.setTunings(ykP, ykI, ykD);
            headingPID.setTunings(hkP, hkI, hkD);

            double xPower = xPID.update(currentPose.getX());
            double yPower = yPID.update(currentPose.getY());
            double headingPower = headingPID.update(currentPose.getHeading());

            Vector2d translationPowers = new Vector2d(xPower, yPower).rotated(-currentPose.getHeading());

            drive.setWeightedDrivePower(
                    new Pose2d(
                            translationPowers,
                            headingPower
                    )
            );

            // Update all state machines
            drive.update();

            // Telemetry
            telemetry.addData("x", currentPose.getX());
            telemetry.addData("y", currentPose.getY());
            telemetry.addData("heading", Math.toDegrees(currentPose.getHeading()));
            telemetry.update();
        }
    }
}

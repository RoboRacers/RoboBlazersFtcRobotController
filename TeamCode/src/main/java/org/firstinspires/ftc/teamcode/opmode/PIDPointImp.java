package org.firstinspires.ftc.teamcode.opmode;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.modules.drive.SampleMecanumDrive;


@Config
@Disabled
@TeleOp(name = "PID to Point", group = "16481-Centerstage")
public class PIDPointImp extends LinearOpMode {

    SampleMecanumDrive drive;
    public static double xkP = 0;
    public static double xkI = 0;
    public static double xkD = 0;
    public static double ykP = 0;
    public static double ykI = 0;
    public static double ykD = 0;
    public static double hkP = 0;
    public static double hkI = 0;
    public static double hkD = 0;

    public static double xTarget = 0;
    public static double yTarget = 0;
    public static double headingTarget = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d());


        while (opModeInInit()) {
        }

        while (!isStopRequested()) {

            PIDFController xPID =  new PIDFController(new PIDCoefficients(xkP,xkI,xkD));
            PIDFController yPID =  new PIDFController(new PIDCoefficients(ykP,ykI,ykD));
            PIDFController headingPID =  new PIDFController(new PIDCoefficients(hkP,hkI,hkD));

            xPID.setTargetPosition(xTarget);
            yPID.setTargetPosition(yTarget);
            headingPID.setTargetPosition(Math.toRadians(headingTarget));

            Pose2d currentPose = drive.getPoseEstimate();

            drive.setWeightedDrivePower(
                    new Pose2d(
                            xPID.update(currentPose.getX()),
                            yPID.update(currentPose.getY()),
                            headingPID.update(currentPose.getHeading())
                    )
            );

            // Update all state machines
            drive.updatePoseEstimate();

            // Telemetry
            telemetry.addLine("\uD83C\uDFCE PID to Point Implementation");
            telemetry.update();
        }
    }
}
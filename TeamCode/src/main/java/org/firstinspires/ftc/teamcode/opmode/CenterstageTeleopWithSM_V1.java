package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.modules.ArmV2;
import org.firstinspires.ftc.teamcode.modules.ArmV2.Link1;

import org.firstinspires.ftc.teamcode.modules.DroneLauncher;
import org.firstinspires.ftc.teamcode.modules.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.modules.trajectorysequence.TrajectorySequence;


@TeleOp
public class CenterstageTeleopWithSM_V1 extends LinearOpMode {
    boolean debug = false;
    RevTouchSensor touchSensor;



    @Override
    public void runOpMode() {

        DcMotorEx liftMotor = hardwareMap.get(DcMotorEx.class, "lift");
        DcMotorEx scissorLift = hardwareMap.get(DcMotorEx.class, "scissor");

        ArmV2 pixelArm = new ArmV2(hardwareMap, telemetry);

        DroneLauncher droneLauncher = new DroneLauncher(hardwareMap, telemetry);

        ArmV2.Link1 links = pixelArm.new Link1(hardwareMap.get(Servo.class, "linkLeft"),
                hardwareMap.get(Servo.class, "linkRight"));

        touchSensor = hardwareMap.get(RevTouchSensor.class, "touch");
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        //drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        TrajectorySequence fwd = drive.trajectorySequenceBuilder(new Pose2d())
                .forward(2)
                .build();
        TrajectorySequence bwd = drive.trajectorySequenceBuilder(new Pose2d())
                .back(2)
                .build();
        TrajectorySequence left = drive.trajectorySequenceBuilder(new Pose2d())
                .strafeLeft(2)
                .build();
        TrajectorySequence right = drive.trajectorySequenceBuilder(new Pose2d())
                .strafeRight(2)
                .build();

        waitForStart();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            gamepad1.left_stick_x, //imperfect strafing fix, must be tuned for new drivetrain
                            -gamepad1.right_stick_x
                    )
            );

            if (gamepad1.dpad_up){
                drive.setPoseEstimate(fwd.start());
                drive.followTrajectorySequence(fwd);
            }
            if (gamepad1.dpad_down){
                drive.setPoseEstimate(bwd.start());
                drive.followTrajectorySequence(bwd);
            }
            if (gamepad1.dpad_left || gamepad2.dpad_right){
                drive.setPoseEstimate(left.start());
                drive.followTrajectorySequence(left);
            }
            if (gamepad1.dpad_right || gamepad2.dpad_left){
                drive.setPoseEstimate(right.start());
                drive.followTrajectorySequence(right);
            }

            if(touchSensor.isPressed()){
                pixelArm.transition(ArmV2.EVENT.PIXEL_PICK_UP);
                debugLog("Is Pressed");
            }

            if(gamepad2.a){
                pixelArm.transition(ArmV2.EVENT.ARM_STOP);
                debugLog("Arm Stop");
            }
            if(gamepad2.left_stick_y < -0.2){
                pixelArm.transition(ArmV2.EVENT.ARM_MOVE_UP);
                debugLog("Arm Up");
            }
            if(gamepad2.dpad_up){
                pixelArm.transition(ArmV2.EVENT.DROP_BACKDROP);
                debugLog("Arm Drop Pos in teleop");
            }
            if(gamepad2.dpad_down){
                pixelArm.transition(ArmV2.EVENT.PIXEL_PICK_UP);
                debugLog("Arm Pick Pos in teleop");
            }
            if(gamepad2.left_bumper){
                pixelArm.transition(ArmV2.EVENT.DROP_LEFT_PIXEL);
                debugLog("Claw Open");
            }
            if(gamepad2.right_bumper){
                pixelArm.transition(ArmV2.EVENT.DROP_RIGHT_PIXEL);
            }
            if(gamepad2.b){
                pixelArm.transition(ArmV2.EVENT.CLAW_CLOSE);
                debugLog("Claw Close");
            }
            if (gamepad2.left_trigger > 0.1){
                pixelArm.transition(ArmV2.EVENT.ARM_INCREMENT_DOWN);
                debugLog("Increment Down");
            }
            if (gamepad2.right_trigger > 0.1){
                pixelArm.transition(ArmV2.EVENT.ARM_INCREMENT_UP);
                debugLog("Increment Up");
            }
            if (gamepad2.x){
                pixelArm.transition(ArmV2.EVENT.DRIVE_WITHOUT_PIXEL_POS);
                //sleep(1000);
                pixelArm.transition(ArmV2.EVENT.DRIVE_WITH_PIXEL_POS);
            }
            if (gamepad2.y){
                links.moveLinks(gamepad2.right_stick_y);
            }
            if (gamepad2.a){
                droneLauncher.launchDrone();
            }
            liftMotor.setPower(gamepad2.right_stick_y);
            scissorLift.setPower(gamepad2.right_stick_x);

            pixelArm.update();
            telemetry.update();
        }
    }

    public void debugLog(String statement){
        if (debug) {
            telemetry.addLine(statement);
        }
    }
}
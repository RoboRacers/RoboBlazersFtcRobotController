package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.modules.ArmV2;
import org.firstinspires.ftc.teamcode.modules.ArmV2.Link1;

import org.firstinspires.ftc.teamcode.modules.DroneLauncher;
import org.firstinspires.ftc.teamcode.modules.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.modules.trajectorysequence.TrajectorySequence;

import java.util.concurrent.TimeUnit;


@TeleOp
public class CenterStageTelopFC_Distance_V3 extends LinearOpMode {
    boolean debug = false;
    RevTouchSensor touchSensor;
    Rev2mDistanceSensor distanceSensor;



    @Override
    public void runOpMode() {

        ArmV2 pixelArm = new ArmV2(hardwareMap, telemetry);

        DroneLauncher droneLauncher = new DroneLauncher(hardwareMap, telemetry);

        ArmV2.Link1 links = pixelArm.new Link1(hardwareMap.get(Servo.class, "linkLeft"),
                hardwareMap.get(Servo.class, "linkRight"));

        ArmV2.Link2 arm = pixelArm.new Link2(hardwareMap.get(DcMotorEx.class,"armMotor"),
                (hardwareMap.get(AnalogInput.class,"pot")));

        DcMotorEx armM = hardwareMap.get(DcMotorEx.class, "armMotor");

        touchSensor = hardwareMap.get(RevTouchSensor.class, "touch");

        distanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "distance");
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        Deadline gamepadRateLimit = new Deadline(500, TimeUnit.MILLISECONDS);

        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        ));
        imu.initialize(parameters);

        DcMotorEx leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        DcMotorEx leftBack = hardwareMap.get(DcMotorEx.class, "leftRear");
        DcMotorEx rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        DcMotorEx rightBack = hardwareMap.get(DcMotorEx.class, "rightRear");

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
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


//            drive.setWeightedDrivePower(
//                    new Pose2d(
//                            -gamepad1.left_stick_y,
//                            gamepad1.left_stick_x, //imperfect strafing fix, must be tuned for new drivetrain
//                            -gamepad1.right_stick_x
//                    )
//            );


            double lx = gamepad1.left_stick_x;
            double ly = gamepad1.left_stick_y;
            double rx = gamepad1.right_stick_x;

            double max = Math.max(Math.abs(lx) + Math.abs(ly) + Math.abs(rx), 1);

            double drivePower = 0.8 - (0.6 * gamepad1.right_trigger);

            if (gamepadRateLimit.hasExpired() && gamepad1.a) {
                imu.resetYaw();
                gamepadRateLimit.reset();
            }

            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double adjustedLx = -ly * Math.sin(heading) + lx * Math.cos(heading);
            double adjustedLy = ly * Math.cos(heading) + lx * Math.sin(heading);

            leftFront.setPower(((adjustedLy - adjustedLx + rx) / max) * drivePower);
            leftBack.setPower(((adjustedLy + adjustedLx + rx) / max) * drivePower);
            rightFront.setPower(((adjustedLy + adjustedLx - rx) / max) * drivePower);
            rightBack.setPower(((adjustedLy - adjustedLx - rx) / max) * drivePower);


            telemetry.addData("distance", distanceSensor.getDistance(DistanceUnit.MM));
            if(distanceSensor.getDistance(DistanceUnit.MM) < 30) {
//                armM.setTargetPosition(armM.getCurrentPosition() - 1);
//                armM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                armM.setPower(1);
                if (arm.getAngle() < 15) {
                    //arm.updateTargetPos(43);
                armM.setTargetPosition(armM.getCurrentPosition() - 10);
                armM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armM.setPower(1);
                }else{

//                    arm.updateTargetPos(arm.getAngle());
//                    pixelArm.update();
                }


            }else{
                if (distanceSensor.getDistance(DistanceUnit.MM) > 60)
                    pixelArm.update();
            }
//            else if (distanceSensor.getDistance(DistanceUnit.MM) > 40 && distanceSensor.getDistance(DistanceUnit.MM) < 260) {
//                arm.updateTargetPos(18);
//            }
//            }else{
//                pixelArm.update();
//            }

            if (gamepad2.share){
                armM.setTargetPosition(armM.getCurrentPosition() + 30);
                armM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armM.setPower(1);            }
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

            //pixelArm.update();
            telemetry.update();
        }
    }

    public void debugLog(String statement){
        if (debug) {
            telemetry.addLine(statement);
        }
    }
}
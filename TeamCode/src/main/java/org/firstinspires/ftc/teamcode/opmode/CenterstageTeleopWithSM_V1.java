package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.modules.ArmV2;
import org.firstinspires.ftc.teamcode.modules.drive.SampleMecanumDrive;


@TeleOp
public class CenterstageTeleopWithSM_V1 extends LinearOpMode {
    boolean debug = false;

    RevTouchSensor touchSensor;



    @Override
    public void runOpMode() {

        ArmV2 pixelArm = new ArmV2(hardwareMap, telemetry);
        touchSensor = hardwareMap.get(RevTouchSensor.class, "touch");
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            gamepad1.left_stick_x, //imperfect strafing fix, must be tuned for new drivetrain
                            gamepad1.right_stick_x
                    )
            );

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
            if(gamepad2.y & gamepad2.right_bumper){
                pixelArm.transition(ArmV2.EVENT.DROP_BOTH_PIXELS);
                debugLog("Claw Open and drop 2 pixels");
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
            if (gamepad2.y){
                pixelArm.transition((ArmV2.EVENT.PID));
            }
            if (gamepad2.x){
                pixelArm.transition(ArmV2.EVENT.DRIVE_WITH_PIXEL_POS);
            }

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
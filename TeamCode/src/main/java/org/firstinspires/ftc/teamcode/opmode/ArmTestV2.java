package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.modules.ArmV2;

@TeleOp
public class ArmTestV2 extends LinearOpMode {
    boolean debug = false;
    @Override
    public void runOpMode() {

        ArmV2 pixelArm = new ArmV2(hardwareMap, telemetry);
        waitForStart();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
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
            if(gamepad2.right_bumper){
                pixelArm.transition(ArmV2.EVENT.DROP_LEFT_PIXEL);
                debugLog("Claw Open");
            }
            if(gamepad2.left_bumper){
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
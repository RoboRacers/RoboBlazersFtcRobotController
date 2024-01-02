package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ClawTestV2 extends LinearOpMode {
    boolean debug = false;
    Servo claw;
    @Override
    public void runOpMode() {

       claw = hardwareMap.get(Servo.class, "clawMotor");
        waitForStart();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if(gamepad2.a){
                claw.setPosition(1.0);
                debugLog("Claw Close");
            }
            if(gamepad2.b){
                claw.setPosition(0.7);
                debugLog("Claw half Open");
            }
            if(gamepad2.y){
                claw.setPosition(0.2);
                debugLog("Claw full Open");
            }

        }
    }

    public void debugLog(String statement){
        if (debug) {
            telemetry.addLine(statement);
            telemetry.update();
        }
    }
}
package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp
public class ClawTestV2 extends LinearOpMode {
    Servo clawLeft;
    Servo clawRight;


    @Override
    public void runOpMode() throws InterruptedException {

        clawLeft = hardwareMap.get(Servo.class, ("linkLeft"));
        clawRight = hardwareMap.get(Servo.class, ("linkRight"));
        clawRight.setDirection(Servo.Direction.REVERSE);


        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("Gamepad value LEFT", gamepad1.left_stick_y);
            telemetry.addData("Gamepad value RIGHT", gamepad1.right_stick_y);
            telemetry.update();
            clawLeft.setPosition(gamepad1.left_stick_y);
            clawRight.setPosition(gamepad1.left_stick_y);
        }
    }
}

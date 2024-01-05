package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp
public class ClawTest extends LinearOpMode {
    Servo linkLeft;
    Servo linkRight;


    @Override
    public void runOpMode() throws InterruptedException {

        linkLeft = hardwareMap.get(Servo.class, ("linkLeft"));
        //linkLeft.setDirection(Servo.Direction.REVERSE);
        linkRight = hardwareMap.get(Servo.class, ("linkRight"));
        //linkRight.setDirection(Servo.Direction.REVERSE);


        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("Gamepad value LEFT", gamepad1.left_stick_y);
            telemetry.addData("Gamepad value RIGHT", gamepad1.right_stick_y);
            telemetry.update();
            linkLeft.setPosition(gamepad1.left_stick_y);
            linkRight.setPosition(gamepad1.left_stick_y);
        }
    }
}

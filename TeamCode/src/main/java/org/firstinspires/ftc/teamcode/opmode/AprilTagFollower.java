package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "AprilTag Mecanum Follower", group = "TeleOp")
public class AprilTagFollower extends LinearOpMode {

    private HuskyLens huskyLens;
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor rearLeftMotor;
    private DcMotor rearRightMotor;

    private final double BASE_SPEED = 0.5; // Base speed for forward movement
    private final double TURN_GAIN = 0.01; // Proportional gain for turning

    @Override
    public void runOpMode() {
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        frontLeftMotor = hardwareMap.get(DcMotor.class, "leftFront");
        frontRightMotor = hardwareMap.get(DcMotor.class, "rightFront");
        rearLeftMotor = hardwareMap.get(DcMotor.class, "leftRear");
        rearRightMotor = hardwareMap.get(DcMotor.class, "rightRear");

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Set motor directions
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rearLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rearRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForStart();

        while (opModeIsActive()) {
            HuskyLens.Block[] blocks = huskyLens.blocks();
            if (blocks.length > 0) {
                // Get the first detected block (assuming only one tag is detected)
                HuskyLens.Block block = blocks[0];
                double xPosition = block.x - 160; // Assuming camera resolution of 320x240
                double turnPower = xPosition * TURN_GAIN;

                // Calculate motor powers
                double frontLeftPower = Range.clip(-BASE_SPEED + turnPower, -1.0, 1.0);
                double frontRightPower = Range.clip(-BASE_SPEED - turnPower, -1.0, 1.0);
                double rearLeftPower = Range.clip(-BASE_SPEED + turnPower, -1.0, 1.0);
                double rearRightPower = Range.clip(-BASE_SPEED - turnPower, -1.0, 1.0);

                // Apply motor powers
                frontLeftMotor.setPower(frontLeftPower);
                frontRightMotor.setPower(frontRightPower);
                rearLeftMotor.setPower(rearLeftPower);
                rearRightMotor.setPower(rearRightPower);
            } else {
                // If no tags detected, stop the robot
                frontLeftMotor.setPower(0);
                frontRightMotor.setPower(0);
                rearLeftMotor.setPower(0);
                rearRightMotor.setPower(0);
            }
        }
    }
}

package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class MecanumDrive_test extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        DcMotorEx frontLeftMotor = hardwareMap.get(DcMotorEx.class, "leftFront");
        DcMotorEx backLeftMotor = hardwareMap.get(DcMotorEx.class, "leftRear");
        DcMotorEx frontRightMotor = hardwareMap.get(DcMotorEx.class, "rightFront");
        DcMotorEx backRightMotor = hardwareMap.get(DcMotorEx.class, "rightRear");

//        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backLeftMotor.setDirection(DcMotorEx.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorEx.Direction.REVERSE);
        //frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
//            double y = -gamepad1.left_stick_y;
//            double x = gamepad1.left_stick_x;
//            double rx = gamepad1.right_stick_x;
//
//
//            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
//            double frontLeftPower = (y + x + rx) / denominator;
//            double backLeftPower = (y - x + rx) / denominator;
//            double frontRightPower = (y - x - rx) / denominator;
//            double backRightPower = (y + x - rx) / denominator;
//
//            frontLeftMotor.setPower(frontLeftPower);
//            backLeftMotor.setPower(backLeftPower);
//            frontRightMotor.setPower(frontRightPower);
//            backRightMotor.setPower(backRightPower);

            double vertical;
            double horizontal;
            double pivot;
            vertical=-gamepad1.left_stick_y;
            horizontal=gamepad1.left_stick_x;
            pivot=gamepad1.right_stick_x;

            if (frontLeftMotor.getPower() == 1 && (frontRightMotor.getPower() == 0 || frontRightMotor.getPower() == 0.9) && backLeftMotor.getPower() == 1 && (backRightMotor.getPower() == 0 || backRightMotor.getPower() == -0.9)){
                frontRightMotor.setPower(0.9);
                backRightMotor.setPower(-0.9);
            }

            if (frontRightMotor.getPower() == -1 && (frontLeftMotor.getPower() == 0 || frontLeftMotor.getPower() == -0.9) && backRightMotor.getPower() == 1 && (backLeftMotor.getPower() == 0 || backLeftMotor.getPower() == 0.9)){
                frontLeftMotor.setPower(-0.9);
                backLeftMotor.setPower(0.9);
            }


            frontRightMotor.setPower (pivot -vertical - horizontal);
            backRightMotor.setPower (-pivot +vertical - horizontal);
            frontLeftMotor.setPower(pivot +vertical - horizontal);
            backLeftMotor.setPower (pivot +vertical + horizontal);

            telemetry.addData("lf-power", frontLeftMotor.getPower());
            telemetry.addData("rf-power", frontRightMotor.getPower());
            telemetry.addData("lb-power", backLeftMotor.getPower());
            telemetry.addData("rb-power", backRightMotor.getPower());

            telemetry.update();
        }
    }
}

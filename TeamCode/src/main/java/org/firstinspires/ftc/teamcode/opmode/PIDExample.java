
package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@Config
@TeleOp
public class PIDExample extends LinearOpMode {

    private DcMotor motor;
    public static double targetPosition = 90; // Target encoder position
    public static double kp = 0.1; // Proportional gain
    public static double ki = 0.01; // Integral gain
    public static double kd = 0.01; // Derivative gain
    public static double integral = 0;
    public static double previousError = 0;

    @Override
    public void runOpMode() {
        motor = hardwareMap.get(DcMotor.class, "armMotor");

        waitForStart();

        while (opModeIsActive()) {
            double currentPosition = motor.getCurrentPosition();
            double error = targetPosition - currentPosition;

            // Proportional term
            double proportional = kp * error;

            // Integral term (with anti-windup)
            integral += ki * error;
            integral = Range.clip(integral, -1 / ki, 1 / ki);

            // Derivative term
            double derivative = kd * (error - previousError);

            // PID output
            double output = proportional + integral + derivative;

            // Apply the output to the motor
            motor.setPower(output);

            // Update previous error
            previousError = error;

            // Check if the motor has reached the target position
            if (Math.abs(error) < 10) {
                motor.setPower(0); // Stop the motor
                break; // Exit the loop
            }

            telemetry.addData("Current Position", currentPosition);
            telemetry.addData("Error", error);
            telemetry.addData("Output", output);
            telemetry.update();
        }
    }
}

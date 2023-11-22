package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@Autonomous(name="PID Arm Control Linear")
public class ArmPIDF  extends LinearOpMode {

    private PIDController controller;

    public static double p =0.005 , i= 0, d=0;
    public static double f = 0;

    public static int target = 100;

    private final double TICKS_IN_DEGREE = 537/360;

    private DcMotorEx armMotor;

    public void runOpMode() {

        controller = new PIDController(p,i,d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        //armMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            controller.setPID(p, i, d);
            int armPos = armMotor.getCurrentPosition();
            telemetry.addData("pos ",armPos);


            double pid = controller.calculate(armPos, target);
            telemetry.addData("pid ",pid);


            double ff = Math.cos(Math.toRadians(target/TICKS_IN_DEGREE)) * f;
            telemetry.addData("ff ",ff);


            double power = pid + ff;

            telemetry.addData("power ",power);
            telemetry.update();
            armMotor.setPower(power);



            //  sleep(5000);  // pause to display final telemetry message.

        }
        // Pace this loop so jaw action is reasonable speed.
    }
}
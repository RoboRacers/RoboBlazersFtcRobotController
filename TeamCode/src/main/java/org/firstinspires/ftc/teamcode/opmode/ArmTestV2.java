package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.modules.ArmV2;

@TeleOp
public class ArmTestV2 extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx pixelArm = null;

    double mult = 0.5;


        @Override
        public void runOpMode() {

            telemetry.addData("Status", "Initialized");
            telemetry.update();

            ArmV2 pixelArm = new ArmV2(hardwareMap, telemetry);
            waitForStart();
            runtime.reset();

            // run until the end of the match (driver presses STOP)
            while (opModeIsActive()) {
                if(gamepad1.a){
                    pixelArm.transition(ArmV2.EVENT.PIXEL_PICK_UP);
                    telemetry.addLine("Move to pixel pick position");
                    telemetry.update();
                }

                // Show the elapsed game time and wheel power.
                telemetry.addData("Status", "Run Time: " + runtime.toString());
                telemetry.update();
            }
        }
}

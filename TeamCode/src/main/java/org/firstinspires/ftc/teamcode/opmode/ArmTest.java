package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.modules.Arm;
@TeleOp
public class ArmTest extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx pixelArm = null;


        @Override
        public void runOpMode() {

            telemetry.addData("Status", "Initialized");
            telemetry.update();

            // Initialize the hardware variables. Note that the strings used here as parameters
            // to 'get' must correspond to the names assigned during the robot configuration
            // step (using the FTC Robot Controller app on the phone).

            //armMotor  = hardwareMap.get(DcMotor.class, "armMotor");


            Arm pixelArm = new Arm(hardwareMap, telemetry);

            // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
            // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
            // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips



            // Wait for the game to start (driver presses PLAY)
            waitForStart();
            runtime.reset();

            // run until the end of the match (driver presses STOP)
            while (opModeIsActive()) {

                if(gamepad2.left_stick_y > 0.2){
                    pixelArm.transition(Arm.EVENT.TWO_LJ_DOWN);
                    telemetry.addLine("Arm Down");
                    telemetry.update();
                }
                if(gamepad2.a){
                    pixelArm.transition(Arm.EVENT.TWO_A);
                    telemetry.addLine("Arm Stopped");
                    telemetry.update();
                }
                if(gamepad2.left_stick_y < -0.2){
                    pixelArm.transition(Arm.EVENT.TWO_LJ_UP);
                    telemetry.addLine("Arm Up");
                    telemetry.update();
                }
                if(gamepad2.right_stick_y > 0.2){
                    pixelArm.transition(Arm.EVENT.TWO_RJ_UP);
                    telemetry.addLine("Link Pick Pos");
                    telemetry.update();
                }
                if(gamepad2.right_stick_y < -0.2) {
                    pixelArm.transition(Arm.EVENT.TWO_RJ_DOWN);
                    telemetry.addLine("Link Drop Pos");
                    telemetry.update();
                }
                if(gamepad2.dpad_up){
                    pixelArm.transition(Arm.EVENT.TWO_DPAD_UP);
                    telemetry.addLine("Arm Drop Pos");
                    telemetry.update();
                }
                if(gamepad2.dpad_down){
                    pixelArm.transition(Arm.EVENT.TWO_DPAD_DOWN);
                    telemetry.addLine("Arm Pick Pos");
                    telemetry.update();
                }


                // Show the elapsed game time and wheel power.
                telemetry.addData("Status", "Run Time: " + runtime.toString());
                telemetry.update();
            }
        }
}

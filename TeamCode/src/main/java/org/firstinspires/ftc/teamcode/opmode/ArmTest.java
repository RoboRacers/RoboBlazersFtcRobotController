package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
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

    double mult = 0.5;


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

//                if(gamepad2.dpad_right){
//                    pixelArm.resetEncoder();
//                }
//
//                //drive
//
//                if (gamepad1.right_bumper){
//                    mult = 1;
//                }else{
//                    mult = 0.5;
//                }
//
//                drive.setWeightedDrivePower(
//                        new Pose2d(
//                                -gamepad1.left_stick_y,
//                                gamepad1.left_stick_x, //imperfect strafing fix, must be tuned for new drivetrain
//                                gamepad1.right_stick_x * mult
//                        )
//                );
//
//                drive.update();


                // arm controls

                if(gamepad2.left_stick_y > 0.2){
                    pixelArm.transition(Arm.EVENT.DROPPING_AT_BACKDROP);
                    //pixelArm.moveArmBackward(-0.4);
                    telemetry.addLine("Arm Down");

                    //telemetry.update();
                }
                if(gamepad2.a){
                    pixelArm.transition(Arm.EVENT.AUTON_START);
                    //pixelArm.moveArmForward(0);
                    //telemetry.addLine("Arm Stopped");
                    //telemetry.update();
                }
                if(gamepad2.left_stick_y < -0.2){
                    pixelArm.transition(Arm.EVENT.DROPPING_AT_TP);
                    //pixelArm.moveArmForward(0.4);
                    telemetry.addLine("Arm Up");
                    //telemetry.update();
                }

                if(gamepad2.y){
                    pixelArm.transition(Arm.EVENT.GO_ARM_GET_POS);
                    telemetry.addData("ARM POS ", pixelArm.getArmPos());
                    telemetry.update();
                }

                if(gamepad2.right_stick_y < 0){
                    if(gamepad2.x)
                        pixelArm.transition(Arm.EVENT.GO_MOVE_LINK_UP);
                    telemetry.addLine("Link Pick Pos");
                    telemetry.update();
                }
                if(gamepad2.right_stick_y > 0) {
                    if(gamepad2.x)
                        pixelArm.transition(Arm.EVENT.GO_MOVE_LINK_DOWN);
                    telemetry.addLine("Link Drop Pos");
                    telemetry.update();
                }
                if(gamepad2.dpad_up){
                    pixelArm.transition(Arm.EVENT.GO_ARM_TO_DROP);
                    //pixelArm.armSetDropPos();
                    telemetry.addLine("Arm Drop Pos");
                    //telemetry.update();
                }
                if(gamepad2.dpad_down){
                    pixelArm.transition(Arm.EVENT.PICK_UP_PIXEL);
                    //pixelArm.armSetPickPos();
                    telemetry.addLine("Arm Pick Pos");
                    //telemetry.update();
                }

                if(gamepad2.a){
                    pixelArm.armStop();
                }

                // claw open close
                if(gamepad2.right_bumper){
                    pixelArm.transition(Arm.EVENT.PICK_UP_PIXEL);
                    //pixelArm.clawOpen();
                    telemetry.addLine("Claw Open");
                    //telemetry.update();
                }
                if(gamepad2.left_bumper){
                    pixelArm.transition(Arm.EVENT.GO_CLAW_DROP_ONE_PIXEL);
                    //pixelArm.clawClose();
                    telemetry.addLine("Claw Close");
                    //telemetry.update();
                }



//                // drone launcher
//                if(gamepad2.a && gamepad2.y) {
//                    dronelauncher.launchDrone();
//                }



                if(gamepad1.left_stick_y != 0 || gamepad1.left_stick_x != 0 || gamepad1.right_stick_x != 0){
                    if (gamepad2.left_bumper && gamepad2.right_bumper){
                        pixelArm.transition(Arm.EVENT.GO_ARM_SAFTEY_MOVE);
                        //pixelArm.safetyMove();
                    }
                }

                if (gamepad2.left_trigger > 0.1){
                    pixelArm.transition(Arm.EVENT.DRIVING_WITHOUT_PIXEL);
                    //pixelArm.incrementDown();
                }
                if (gamepad2.right_trigger > 0.1){
                    pixelArm.transition(Arm.EVENT.ALLIGN_WITH_PIXEL);
                    //pixelArm.incrementUp();
                }

                if (gamepad2.y){
                    //pixelArm.linkPickPos();
                }

                if (gamepad2.dpad_left){
                    //pixelArm.overideSafety();
                }


                // Show the elapsed game time and wheel power.
                telemetry.addData("Status", "Run Time: " + runtime.toString());
                telemetry.update();
            }
        }
}

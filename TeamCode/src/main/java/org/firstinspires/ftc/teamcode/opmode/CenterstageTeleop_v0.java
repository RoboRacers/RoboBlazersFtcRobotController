/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.modules.Arm;
import org.firstinspires.ftc.teamcode.modules.DroneLauncher;
import org.firstinspires.ftc.teamcode.modules.StateMachines.ArmStates;
import org.firstinspires.ftc.teamcode.modules.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.modules.StateMachines.ArmStates;
import org.firstinspires.ftc.teamcode.modules.Lift;



import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.modules.Arm;
import org.firstinspires.ftc.teamcode.modules.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.modules.trajectorysequence.TrajectorySequence;


/*
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Roboblazers Teleop v0", group="Linear OpMode")
public class CenterstageTeleop_v0 extends LinearOpMode {

    // Declare OpMode members.

    double mult;
    double mult1;
    private ElapsedTime runtime = new ElapsedTime();

    DcMotorEx Lift;

    RevTouchSensor touchSensor;

    static boolean isPressed;

    //private DcMotorEx pixelArm = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // create objects
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        touchSensor = hardwareMap.get(RevTouchSensor.class, "touch");

        Arm pixelArm = new Arm(hardwareMap, telemetry);
        DroneLauncher dronelauncher = new DroneLauncher(hardwareMap, telemetry);

        Lift = hardwareMap.get(DcMotorEx.class, "Lift");
        //Lift lift = new Lift(hardwareMap, telemetry);


        TrajectorySequence right = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(0)))// 90 is facing red
                .strafeRight(1)
                .build();

        TrajectorySequence left = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(0)))// 90 is facing red
                .strafeLeft(1)
                .build();

        TrajectorySequence forward = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(0)))// 90 is facing red
                .forward(1)
                .build();

        TrajectorySequence back = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(0)))// 90 is facing red
                .back(1)
                .build();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            isPressed = touchSensor.isPressed();

            if (isPressed){
                pixelArm.resetEncoder();
                pixelArm.armSetPickPos();
                telemetry.addLine("TOUCH PRESSED");
                telemetry.update();
            }

//            if(gamepad2.dpad_right){
//                pixelArm.resetEncoder();
//            }

            //drive

            if (gamepad1.right_bumper){
                mult = 1;
            }else{
                mult = 0.5;
            }

            if (gamepad1.left_bumper){
                mult1 = 0.5;
            } else{
                mult1 = 1;
            }

            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y * mult1,
                            gamepad1.left_stick_x * mult1, //imperfect strafing fix, must be tuned for new drivetrain
                            gamepad1.right_stick_x * mult
                    )
            );

            drive.update();

            if (gamepad1.dpad_up){
                drive.setPoseEstimate(forward.start());

                drive.followTrajectorySequence(forward);
            } else if (gamepad1.dpad_down) {
                drive.setPoseEstimate(back.start());

                drive.followTrajectorySequence(back);
            } else if (gamepad1.dpad_left) {
                drive.setPoseEstimate(left.start());

                drive.followTrajectorySequence(left);
            } else if (gamepad1.dpad_right) {
                drive.setPoseEstimate(right.start());

                drive.followTrajectorySequence(right);
            }


            // arm controls

            if(gamepad2.left_stick_y > 0.2){
                pixelArm.moveArmBackward(-0.4);
                telemetry.addLine("Arm Down");

                //telemetry.update();
            }
            if(gamepad2.a){
                pixelArm.moveArmForward(0);
                //telemetry.addLine("Arm Stopped");
                //telemetry.update();
            }
            if(gamepad2.left_stick_y < -0.2){
                pixelArm.moveArmForward(0.4);
                telemetry.addLine("Arm Up");
                //telemetry.update();
            }

            if(gamepad2.y){
                telemetry.addData("ARM POS ", pixelArm.getArmPos());
                telemetry.update();
            }

            if(gamepad2.right_stick_y < 0){
                if(gamepad2.x)
                    pixelArm.moveLinkPickUp(gamepad2.right_stick_y);
                telemetry.addLine("Link Pick Pos");
                telemetry.update();
            }
            if(gamepad2.right_stick_y > 0) {
                if(gamepad2.x)
                    pixelArm.moveLinkDrop(gamepad2.right_stick_y);
                telemetry.addLine("Link Drop Pos");
                telemetry.update();
            }
            if(gamepad2.dpad_up){
                pixelArm.armSetDropPos();
                telemetry.addLine("Arm Drop Pos");
                telemetry.update();
            }
            if(gamepad2.dpad_down){
                pixelArm.moveArmBackward(-0.3);
//                sleep(1000);
//                pixelArm.armSetPickPos();
                telemetry.addLine("Arm Pick Pos");
                //telemetry.update();
            }

            if(gamepad2.a){
                pixelArm.armStop();
            }

            // claw open close
            if(gamepad2.right_bumper){
                pixelArm.clawOpen();
                telemetry.addLine("Claw Open");
                //telemetry.update();
            }
            if(gamepad2.left_bumper){
                pixelArm.clawClose();
                telemetry.addLine("Claw Close");
                //telemetry.update();
            }



            // drone launcher
            if(gamepad2.a && gamepad2.y) {
                dronelauncher.launchDrone();
            }



            if(gamepad1.left_stick_y != 0 || gamepad1.left_stick_x != 0 || gamepad1.right_stick_x != 0){
                if (gamepad2.left_bumper && gamepad2.right_bumper){
                    pixelArm.safetyMove();

                }
            }

            if (gamepad2.left_trigger > 0.1){
                pixelArm.incrementDown();
            }
            if (gamepad2.right_trigger > 0.1){
                pixelArm.incrementUp();
            }

//            if (gamepad2.y){
//                pixelArm.linkPickPos();
//            }

//            if (gamepad2.dpad_left){
//                pixelArm.overideSafety();
//            }

            if (gamepad2.dpad_left){
                Lift.setPower(0.5);
            }

            if (gamepad2.dpad_right){
                Lift.setPower(0.2);
            }

            if (gamepad2.x){
                Lift.setPower(0);
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            //telemetry.update();
        }
    }
}

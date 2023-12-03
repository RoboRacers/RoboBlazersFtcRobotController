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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.modules.Arm;
import org.firstinspires.ftc.teamcode.modules.DroneLauncher;
import org.firstinspires.ftc.teamcode.modules.StateMachines.ArmStates;
import org.firstinspires.ftc.teamcode.modules.drive.SampleMecanumDrive;



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

@TeleOp(name="Roboblazers Teleop v1", group="Linear OpMode")
public class CenterstageTeleop_v1 extends LinearOpMode {

    // Declare OpMode members.

    double mult;
    private ElapsedTime runtime = new ElapsedTime();


    //private DcMotorEx pixelArm = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // create objects
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Arm pixelArm = new Arm(hardwareMap, telemetry);
        ArmStates armStates = new ArmStates(pixelArm);

        armStates.transition(ArmStates.EVENT.AUTON_START);

        //ArmStates.EVENT = AUTON_START;





        DroneLauncher dronelauncher = new DroneLauncher(hardwareMap, telemetry);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if(gamepad2.dpad_right){
                pixelArm.resetEncoder();
                //ArmStates.EVENT.
            }

            //drive

            if (gamepad1.right_bumper){
                mult = 1;
            }else{
                mult = 0.5;
            }

            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            gamepad1.left_stick_x, //imperfect strafing fix, must be tuned for new drivetrain
                            gamepad1.right_stick_x * mult
                    )
            );

            drive.update();


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
                //telemetry.update();
            }
            if(gamepad2.dpad_down){
                pixelArm.armSetPickPos();
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

            if (gamepad2.y){
                pixelArm.linkPickPos();
            }

            if (gamepad2.dpad_left){
                pixelArm.overideSafety();
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            //telemetry.update();
        }
    }
}

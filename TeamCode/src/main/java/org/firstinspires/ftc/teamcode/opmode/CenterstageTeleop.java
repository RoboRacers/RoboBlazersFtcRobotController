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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.modules.Arm;
import org.firstinspires.ftc.teamcode.modules.DroneLauncher;


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

@TeleOp(name="Roboblazers Teleop", group="Linear OpMode")
public class CenterstageTeleop extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    //private DcMotorEx pixelArm = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // create objects
        Arm pixelArm = new Arm(hardwareMap, telemetry);
        DroneLauncher dronelauncher = new DroneLauncher(hardwareMap, telemetry);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // arm controls
            if(gamepad2.left_stick_y > 0.2){
                pixelArm.transition(Arm.EVENT.TWO_LJ_DOWN);
                //pixelArm.moveArmBackward(-0.2);
                telemetry.addLine("Arm Down");
                telemetry.update();
            }
            if(gamepad2.a){
                pixelArm.transition(Arm.EVENT.TWO_A);
                //pixelArm.moveArmForward(0);
                telemetry.addLine("Arm Stopped");
                telemetry.update();
            }
            if(gamepad2.left_stick_y < -0.2){
                pixelArm.transition(Arm.EVENT.TWO_LJ_UP);
                //.moveArmForward(0.2);
                telemetry.addLine("Arm Up");
                telemetry.update();
            }
            if(gamepad2.right_stick_y > 0.2){
                pixelArm.transition(Arm.EVENT.TWO_RJ_DOWN);
                //pixelArm.moveLinkPickUp();
                telemetry.addLine("Link Pick Pos");
                telemetry.update();
            }
            if(gamepad2.right_stick_y < -0.2) {
                pixelArm.transition(Arm.EVENT.TWO_RJ_UP);
                //pixelArm.moveLinkDrop();
                telemetry.addLine("Link Drop Pos");
                telemetry.update();
            }
            if(gamepad2.dpad_up){
                pixelArm.transition(Arm.EVENT.TWO_DPAD_UP);
                //pixelArm.armSetDropPos();
                telemetry.addLine("Arm Drop Pos");
                telemetry.update();
            }
            if(gamepad2.dpad_down){
                pixelArm.transition(Arm.EVENT.TWO_DPAD_DOWN);
                //pixelArm.armSetPickPos();
                telemetry.addLine("Arm Pick Pos");
                telemetry.update();
            }

            // claw open close
            if(gamepad2.right_bumper){
                pixelArm.transition(Arm.EVENT.TWO_RB);
                //pixelArm.clawOpen();
                telemetry.addLine("Claw Open");
                telemetry.update();
            }
            if(gamepad2.left_bumper){
                pixelArm.transition(Arm.EVENT.TWO_LB);
                //pixelArm.clawClose();
                telemetry.addLine("Claw Close");
                telemetry.update();
            }

            // drone launcher
            if(gamepad2.a && gamepad2.dpad_up) {
                dronelauncher.launchDrone();
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}

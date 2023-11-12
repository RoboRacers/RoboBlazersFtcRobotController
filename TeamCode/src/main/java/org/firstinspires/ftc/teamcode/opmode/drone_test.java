package org.firstinspires.ftc.teamcode.opmode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.modules.drone_launcher;

public class drone_test extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private Servo droneServo = null;


    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        droneServo  = hardwareMap.get(Servo.class, "droneServo");

        drone_launcher dronelauncher = new drone_launcher(hardwareMap);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if(gamepad2.a && gamepad2.dpad_up) {
                dronelauncher.launchDrone();
            }
        }
    }

}

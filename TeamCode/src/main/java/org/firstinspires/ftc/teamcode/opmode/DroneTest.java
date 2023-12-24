package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.modules.DroneLauncher;

@TeleOp
public class DroneTest extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    // private Servo droneServo = null;


    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        //droneServo  = hardwareMap.get(Servo.class, "droneServo");

        DroneLauncher dronelauncher = new DroneLauncher(hardwareMap, telemetry);

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

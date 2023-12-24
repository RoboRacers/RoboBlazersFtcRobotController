package org.firstinspires.ftc.teamcode.modules;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DroneLauncher {
    Servo tempServo;
    Servo droneServo;
    public enum STATE {
        Drone_Launched,
    }

    public enum EVENT{
        Drone_Button_Pressed
    }

    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    double LAUNCH_POSITION = 1.0;
    double START_POSITION = 0.3;
    double INCREMENT_POSITION = 0.05;



    public DroneLauncher(HardwareMap hardwareMap, Telemetry telemetry) {
        droneServo = hardwareMap.get(Servo.class, "droneServo");
        //droneServo.setPosition(START_POSITION);
    }

    public void launchDrone() {
        timer.reset();
        double target = timer.time() + 5;
        double current_position = START_POSITION;
        while(current_position <= LAUNCH_POSITION){
            if(timer.time()>target){
                current_position = current_position + INCREMENT_POSITION;
                target = timer.time() + 5;
                droneServo.setPosition(current_position);
            }
        }
    }
}

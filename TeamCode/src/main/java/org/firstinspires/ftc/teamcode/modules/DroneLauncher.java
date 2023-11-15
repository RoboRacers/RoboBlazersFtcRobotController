package org.firstinspires.ftc.teamcode.modules;

import static org.firstinspires.ftc.robotcontroller.external.samples.ConceptRampMotorSpeed.INCREMENT;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DroneLauncher extends LinearOpMode {
    Servo tempServo;
    Servo droneServo;

    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);



    public DroneLauncher(HardwareMap hardwareMap, Telemetry telemetry) {
        droneServo = hardwareMap.get(Servo.class, "droneServo");
    }

    public void launchDrone() {
        double target = timer.time() + 5;
        while(opModeIsActive()){
            if(timer.time()>target){
                target += 5;
                double position = INCREMENT;
                droneServo.setPosition(position);
            }
        }
    }

    @Override
    public void runOpMode(){}
}

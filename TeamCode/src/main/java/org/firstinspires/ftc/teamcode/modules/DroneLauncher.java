package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class DroneLauncher {
    Servo tempServo;
    Servo droneServo;




    public DroneLauncher(HardwareMap hardwareMap) {
        droneServo = hardwareMap.get(Servo.class, "droneServo");
    }

    public void launchDrone() {
        tempServo.setPosition(0.5);
    }
}

package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class drone_launcher {
    Servo tempServo;
    Servo droneServo;

    public void drone_launcher(HardwareMap hardwareMap) {
        droneServo = hardwareMap.get(Servo.class, "droneServo");
    }

    public void launchDrone() {
        tempServo.setPosition(0.5);
    }
}

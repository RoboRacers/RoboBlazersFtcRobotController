package org.firstinspires.ftc.teamcode.modules.StateMachines;

import org.firstinspires.ftc.teamcode.modules.Arm;

public class DroneStates {
    public enum STATE {
        Drone_Launched
    }

    public enum EVENT{
        Drone_Button_Pressed
    }
    STATE currentState;
    EVENT currentEvent;

    public void transition(EVENT event) {
        switch (event) {
            case Drone_Button_Pressed:
                currentState = STATE.Drone_Launched;
                break;
        }
    }
}

package org.firstinspires.ftc.teamcode.modules;

public class drone_launcher {
    public enum STATE {
        DRONE_LAUNCHER_LOADED,
        DRONE_LAUNCHED,
    }

    public enum EVENT {
        GAME_START,
        DRONE_LAUNCH_BUTTON_PRESSED,
    }

    STATE currentState;

    public drone_launcher.STATE getState() {
        return currentState;
    }

    public void transition(EVENT event) {
        switch (event) {
            case GAME_START:
                currentState = drone_launcher.STATE.DRONE_LAUNCHER_LOADED;
                break;
            case DRONE_LAUNCH_BUTTON_PRESSED:
                currentState = drone_launcher.STATE.DRONE_LAUNCHED;
                break;
        }
    }
}

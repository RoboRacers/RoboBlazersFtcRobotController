package org.firstinspires.ftc.teamcode.modules;

public class lift {
    public enum STATE {
        LIFT_OPEN_UP,
        LIFT_GRABBED_TRUSS,
        LIFT_RETRACTED,

    }

    public enum EVENT {
        IS_ENDGAME,
        ROBOT_UNDER_TRUSS,
        ROBOT_HANGING,
    }


    lift.STATE currentState;

    public lift.STATE getState() {
        return currentState;
    }

    public void transition(EVENT event) {
        switch (event) {
            case IS_ENDGAME:
                currentState = lift.STATE.LIFT_OPEN_UP;
                break;
            case ROBOT_UNDER_TRUSS:
                currentState = lift.STATE.LIFT_GRABBED_TRUSS;
                break;
            case ROBOT_HANGING:
                currentState = lift.STATE.LIFT_RETRACTED;
                break;
        }
    }
}

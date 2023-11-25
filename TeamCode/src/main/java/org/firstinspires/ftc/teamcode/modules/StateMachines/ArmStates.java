package org.firstinspires.ftc.teamcode.modules.StateMachines;

import org.firstinspires.ftc.robotcore.external.StateMachine;
import org.firstinspires.ftc.teamcode.modules.Arm;

public class ArmStates extends StateMachine {
    Arm myArm;
    STATE currentState;
    public enum STATE {

        //Arm States

        ARM_HOLD_POS,
        ARM_START,
        ARM_MOVE_FOR_DRIVING,
        ARM_MOVE_UP,
        ARM_MOVE_DOWN,
        ARM_INCREMENT_UP,
        ARM_INCREMENT_DOWN,
        ARM_MOVE_PICK,
        ARM_MOVE_DROP,
        ARM_MOVE_DROP_AUTON,
        ARM_SAFETY_MOVE,
        ARM_RESET_ENCODER, //might be removed or changed to POT
        ARM_GET_POS,

        //Link States

        MOVE_LINK_UP,
        MOVE_LINK_DOWN,

        //Claw States

        CLAW_OPEN,
        CLAW_CLOSE,
        CLAW_DROP_ONE_PIXEL
    }

    public enum EVENT{
        AUTON_START,
        DETECTING_TP,
        DRIVING_WITH_PIXEL,
        DROPPING_AT_TP,
        DROPPING_AT_BACKDROP,
        DRIVING_WITHOUT_PIXEL,
        ALLIGN_WITH_PIXEL,
        PICK_UP_PIXEL
    }

    public ArmStates(Arm myArm){
        this.myArm = myArm;
    }

    public ArmStates.STATE getState(){
        return currentState;
    }

    public void transition(ArmStates.EVENT event) {
        switch (event) {
            case AUTON_START:
                currentState = STATE.ARM_START;
                break;
            case DETECTING_TP:
                currentState = STATE.ARM_HOLD_POS;
                break;
            case DRIVING_WITH_PIXEL:
                currentState = STATE.ARM_MOVE_FOR_DRIVING;
                break;
            case DROPPING_AT_TP:
                currentState = STATE.ARM_MOVE_DROP_AUTON;
                break;
            case DROPPING_AT_BACKDROP:
                currentState = STATE.ARM_MOVE_DROP;
                break;
            case DRIVING_WITHOUT_PIXEL:
                currentState = STATE.ARM_MOVE_FOR_DRIVING;
                break;
            case ALLIGN_WITH_PIXEL:
                currentState = STATE.ARM_MOVE_PICK;
                break;
            case PICK_UP_PIXEL:
                currentState = STATE.CLAW_CLOSE;
        }
    }
    public void update() {
        switch (currentState){
            case ARM_START:
                //myArm.startPosInAuton(-200);
        }
    }
}



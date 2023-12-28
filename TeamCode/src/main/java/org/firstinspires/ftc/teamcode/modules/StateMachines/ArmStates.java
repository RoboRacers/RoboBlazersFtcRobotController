package org.firstinspires.ftc.teamcode.modules.StateMachines;

import org.firstinspires.ftc.robotcore.external.StateMachine;
import org.firstinspires.ftc.teamcode.modules.Arm;

public class ArmStates extends StateMachine {
    Arm myArm;
    STATE currentState;
    public enum STATE {

        //Arm States

        ARM_FOLDED_POS,
        ARM_DROPPING_BACKDROP,
        ARM_MOVING_UP,
        ARM_MOVING_DOWN,
        ARM_INCREMENT_UP,
        ARM_INCREMENT_DOWN,
        LINK_MOVE_BACK,
        LINK_MOVE_FORWARD,
        CLAW_DROPPED_BOTTOM_PIXEL,
        CLAW_DROPPED_TOP_PIXEL,
        CLAW_CLOSED,
        CLAW_OPEN,
    }

    public enum EVENT{
        NEED_ARM_TO_FOLD,
        NEED_DROP,
        NEED_ARM_UP,
        NEED_ARM_DOWN,
        NEED_ARM_INCREMENT_UP,
        NEED_ARM_INCREMENT_DOWN,
        CLAW_LINK_DROPPING,
        CLAW_LINK_PICKUP,
        DROPPING_BOTTOM_PIXEL,
        DROPPING_TOP_PIXEL,
        THE_CLAW_IS_CLOSED,
        THE_CLAW_IS_OPEN,
    }

    public ArmStates(Arm myArm){
        this.myArm = myArm;
    }

    public ArmStates.STATE getState(){
        return currentState;
    }

    public void transition(ArmStates.EVENT event) {
        switch (event) {
            case NEED_ARM_TO_FOLD:
                currentState = STATE.ARM_FOLDED_POS;
                break;
            case NEED_DROP:
                currentState = STATE.ARM_DROPPING_BACKDROP;
                break;
            case NEED_ARM_UP:
                currentState = STATE.ARM_MOVING_UP;
                break;
            case NEED_ARM_DOWN:
                currentState = STATE.ARM_MOVING_DOWN;
                break;
            case NEED_ARM_INCREMENT_UP:
                currentState = STATE.ARM_INCREMENT_UP;
                break;
            case NEED_ARM_INCREMENT_DOWN:
                currentState = STATE.ARM_INCREMENT_DOWN;
                break;
            case CLAW_LINK_DROPPING:
                currentState = STATE.LINK_MOVE_BACK;
            case CLAW_LINK_PICKUP:
                currentState = STATE.LINK_MOVE_FORWARD;
            case DROPPING_BOTTOM_PIXEL:
                currentState = STATE.CLAW_DROPPED_BOTTOM_PIXEL;
            case DROPPING_TOP_PIXEL:
                currentState = STATE.CLAW_DROPPED_TOP_PIXEL;
            case THE_CLAW_IS_CLOSED:
                currentState = STATE.CLAW_CLOSED;
            case THE_CLAW_IS_OPEN:
                currentState = STATE.CLAW_OPEN;
        }
    }
    public void update() {
        switch (currentState){
            case ARM_FOLDED_POS:
                myArm.armSetPickPos();
            case ARM_DROPPING_BACKDROP:
                myArm.armSetDropPos();
            case ARM_MOVING_UP:
                myArm.moveArmForward(0.4);
            case ARM_MOVING_DOWN:
                myArm.moveArmBackward(-0.4);
            case ARM_INCREMENT_UP:
                myArm.incrementUp();
            case ARM_INCREMENT_DOWN:
                myArm.incrementDown();
            case LINK_MOVE_BACK:
                //myArm.moveLinkDrop(gamepad2.right_stick_y);
            case LINK_MOVE_FORWARD:
                //myArm.moveLinkPickUp(gamepad2.right_stick_y);
            case CLAW_DROPPED_BOTTOM_PIXEL:
                myArm.clawDropBottom();
            case CLAW_DROPPED_TOP_PIXEL:
                myArm.clawDropTop();
            case CLAW_CLOSED:
                myArm.clawClose();
            case CLAW_OPEN:
                myArm.clawOpen();

        }
    }
}



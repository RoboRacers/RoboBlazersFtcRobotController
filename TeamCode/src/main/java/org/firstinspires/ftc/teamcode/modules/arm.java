package org.firstinspires.ftc.teamcode.modules;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class arm {


    DcMotorEx armMotor;
    Servo linkMotor;
    Servo clawMotor;
    private ElapsedTime runtime = new ElapsedTime();
    static final double     COUNTS_PER_MOTOR_REV    = 5281.1 ;
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION);
    static final double     DRIVE_SPEED             = 0.6;
    boolean ARM_MOTOR_BUSY = false;


    public enum STATE {
        ARM_FORWARD,
        ARM_BACKWARD,
        DROP_POS,
        PICK_POS,
        ARM_LINK1_FORWARD,
        ARM_LINK1_BACK,
        CLAW_OPEN,
        CLAW_CLOSE,
        ARM_IDLE,
    }

    public enum EVENT {
        TWO_LJ_DOWN,
        TWO_LJ_UP,
        TWO_Y,
        TWO_A,
        TWO_RJ_UP,
        TWO_RJ_DOWN,
        TWO_LB,
        TWO_RB
    }

    STATE currentState;
    STATE armcurrentState;

    public arm.STATE getState() {
        return currentState;
    }

    public void transition(EVENT event) {
        switch (event) {
            case TWO_LJ_DOWN:
                currentState = arm.STATE.ARM_FORWARD;
                break;
            case TWO_LJ_UP:
                currentState = arm.STATE.ARM_BACKWARD;
                break;
            case TWO_Y:
                currentState = arm.STATE.DROP_POS;
                break;
            case TWO_A:
                currentState = arm.STATE.PICK_POS;
                break;
            case TWO_RJ_UP:
                currentState = arm.STATE.ARM_LINK1_FORWARD;
                break;
            case TWO_RJ_DOWN:
                currentState = arm.STATE.ARM_LINK1_BACK;
                break;
            case TWO_LB:
                currentState = arm.STATE.CLAW_OPEN;
                break;
            case TWO_RB:
                currentState = arm.STATE.CLAW_CLOSE;
                break;
        }
    }

    public void update(){
        switch (currentState) {
            case ARM_FORWARD:
                break;
            case ARM_BACKWARD:
                break;
            case ARM_LINK1_FORWARD:
                break;
            case ARM_LINK1_BACK:
                break;
        }
    }

    public arm(HardwareMap hardwareMap) {
        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
    }


    public void moveArmForward(double power){
        if (power > 0){
            return;
        } else if (power <= 0) {
            armMotor.setPower(power);
        }
    }

    public void moveArmBackward(double power){
        if (power < 0){
            return;
        } else if (power >= 0) {
            armMotor.setPower(power);
        }
    }

    public void armSetPickPos(){

        if(armcurrentState == STATE.PICK_POS || armcurrentState == STATE.ARM_FORWARD) /*And check if it is within limits **/{
            return;
        }
        else if(armcurrentState == STATE.DROP_POS ||
                armcurrentState == STATE.ARM_BACKWARD){
            armMotor.setPower(0);
            armcurrentState = STATE.ARM_IDLE;
        }
        clawOpen();
        encoderDrive(0.05, 0);
        System.out.println("completed set pick pos");
    }
    public void armSetDropPos(){
        if(armcurrentState == STATE.DROP_POS || armcurrentState == STATE.ARM_BACKWARD) /* And check if it is within limits **/{
            return;
        }
        else if(armcurrentState == STATE.PICK_POS ||
                armcurrentState == STATE.ARM_FORWARD){
            armMotor.setPower(0);
            armcurrentState = STATE.ARM_IDLE;
        }
        clawClose();
        encoderDrive(0.05, 120);
        System.out.println("completed set drop pos");
    }
    public void moveLinkPickUp(){
        linkMotor.setPosition(0);
    }
    public void moveLinkDrop(){
        linkMotor.setPosition(1);
    }
    public void clawOpen(){
        clawMotor.setPosition(0.75);
    }
    public void clawClose(){
        clawMotor.setPosition(0.25);
    }



    // 120 degrees --> need to travel 1760 ticks
    public void encoderDrive(double speed, double turnAngle) {
        int newArmTarget;


        // Determine new target position, and pass to motor controller
        newArmTarget = armMotor.getCurrentPosition() + (int) (turnAngle * COUNTS_PER_INCH);
        armMotor.setTargetPosition(newArmTarget);

        // Turn On RUN_TO_POSITION
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        runtime.reset();
        armMotor.setPower(Math.abs(speed));

        ARM_MOTOR_BUSY = true;

        /**
        // Stop all motion;
        armMotor.setPower(0);

        // Turn off RUN_TO_POSITION
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sleep(250);   // optional pause after each move.
         **/
    }
}

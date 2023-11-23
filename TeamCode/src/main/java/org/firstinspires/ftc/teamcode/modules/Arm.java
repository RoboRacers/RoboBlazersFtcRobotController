package org.firstinspires.ftc.teamcode.modules;

import static java.lang.Thread.sleep;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Arm {

    Telemetry myTelemetry;
    DcMotorEx pixelArm;
    Servo linkMotor;
    Servo clawMotor;

    double dropPos = 1969;

    double pickPos = 2950;

    double linkPos = 0.66;
    private ElapsedTime runtime = new ElapsedTime();
    static final double     COUNTS_PER_MOTOR_REV    = 5281.1 ;
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION);
    static final double     DRIVE_SPEED             = 0.6;
    boolean ARM_MOTOR_BUSY = false;

    private PIDController controller;

    public static double p =0.005 , i= 0, d=0;
    public static double f = 0;

    public static int target = 100;

    private final double TICKS_IN_DEGREE = 537/360;

    private DcMotorEx armMotor;



//    public void setDirection(DcMotorSimple.Direction reverse) {
//    }


    public enum STATE {
        ARM_FORWARD,
        ARM_BACKWARD,
        DROP_POS,
        PICK_POS,
        ARM_LINK1_PICK,
        ARM_LINK1_DROP,
        CLAW_OPEN,
        CLAW_CLOSE,
        ARM_IDLE,
    }

    public enum EVENT {
        TWO_LJ_DOWN,
        TWO_LJ_UP,
        TWO_DPAD_UP,
        TWO_DPAD_DOWN,
        TWO_RJ_UP,
        TWO_RJ_DOWN,
        TWO_LB,
        TWO_RB,
        TWO_A
    }

    STATE currentState;
    STATE armcurrentState;

    public Arm.STATE getState() {
        return currentState;
    }

    public void transition(EVENT event) {
        switch (event) {
            case TWO_LJ_DOWN:
                currentState = Arm.STATE.ARM_FORWARD;
                break;
            case TWO_LJ_UP:
                currentState = Arm.STATE.ARM_BACKWARD;
                break;
            case TWO_DPAD_UP:
                currentState = Arm.STATE.DROP_POS;
                break;
            case TWO_DPAD_DOWN:
                currentState = Arm.STATE.PICK_POS;
                break;
            case TWO_RJ_UP:
                currentState = Arm.STATE.ARM_LINK1_PICK;
                break;
            case TWO_RJ_DOWN:
                currentState = Arm.STATE.ARM_LINK1_DROP;
                break;
            case TWO_LB:
                currentState = Arm.STATE.CLAW_OPEN;
                break;
            case TWO_RB:
                currentState = Arm.STATE.CLAW_CLOSE;
                break;
        }
    }

    public void update(){
        switch (currentState) {
            case ARM_FORWARD:
                moveArmForward(0);
                break;
            case ARM_BACKWARD:
                moveArmBackward(-0.2);
                break;
//            case ARM_LINK1_PICK:
//                moveLinkPickUp();
//                break;
//            case ARM_LINK1_DROP:
//                moveLinkDrop();
//                break;
            case DROP_POS:
                armSetDropPos();
                break;
            case PICK_POS:
                armSetPickPos();
                break;
            case CLAW_CLOSE:
                clawClose();
                break;
            case CLAW_OPEN:
                clawOpen();
                break;
        }
    }

    public Arm(HardwareMap hardwareMap, Telemetry telemetry) {
        pixelArm = hardwareMap.get(DcMotorEx.class, "armMotor");
        pixelArm.setDirection(DcMotorSimple.Direction.REVERSE);
        pixelArm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        pixelArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        linkMotor = hardwareMap.get(Servo.class, "linkMotor");
        //linkMotor.setPosition(0.66);
        clawMotor = hardwareMap.get(Servo.class, "clawMotor");
        myTelemetry = telemetry;

        controller = new PIDController(p,i,d);
        //armMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

    }


    public void resetEncoder(){
        pixelArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    public void moveArmPIDF(){
        controller.setPID(p, i, d);
        int armPos = pixelArm.getCurrentPosition();


        double pid = controller.calculate(armPos, target);


        double ff = Math.cos(Math.toRadians(target/TICKS_IN_DEGREE)) * f;

        double power = pid + ff;

        pixelArm.setPower(power);

    }

    public void moveArmForward(double power){
        if (power < 0){
            return;
        } else if (power >= 0) {
            pixelArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            pixelArm.setPower(power);
        }
    }

    public void moveArmBackward(double power){
        if (power > 0){
            return;
        } else if (power <= 0) {
            pixelArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            pixelArm.setPower(power);
        }
    }

    public void armSetPickPos(){

//        if(armcurrentState == STATE.PICK_POS || armcurrentState == STATE.ARM_FORWARD) /*And check if it is within limits **/{
//            return;
//        }
//        else if(armcurrentState == STATE.DROP_POS ||
//                armcurrentState == STATE.ARM_BACKWARD){
//            pixelArm.setPower(-0.20);
//            armcurrentState = STATE.ARM_IDLE;
//        }
//        clawOpen();

        pixelArm.setTargetPosition((int) pickPos);

        pixelArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pixelArm.setPower(0.4);
        linkMotor.setPosition(linkPos);


    }


    public void armSetDropPos(){
        pixelArm.setTargetPosition((int) dropPos);
        pixelArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pixelArm.setPower(0.5);

        linkMotor.setPosition(linkPos);
    }

    public void armStop(){
        pixelArm.setPower(0);
//    }
//
//    public void armSetDropPos(){
//        if(armcurrentState == STATE.DROP_POS || armcurrentState == STATE.ARM_BACKWARD) /* And check if it is within limits **/{
//            return;
//        }
//        else if(armcurrentState == STATE.PICK_POS ||
//                armcurrentState == STATE.ARM_FORWARD){
//            pixelArm.setPower(0);
//            armcurrentState = STATE.ARM_IDLE;
//        }
//        clawClose();
//        encoderDrive(0.05, 120);

    }

    public int getArmPos(){
        return pixelArm.getCurrentPosition();
    }

    public void moveLinkPickUp(double position){
        //double output = 0.4 + ((1.0 - 0.4) / (1 - -1)) * (position - -1);
        double output = 0 + ((1.0 - 0) / (1 - -1)) * (position - -1);

        myTelemetry.addData("Ranged Output", output);
        //myTelemetry.update();
        linkMotor.setPosition(output);
    }
    public void moveLinkDrop(double position){

        //double output = 0.4 + ((1.0 - 0.4) / (1 - -1)) * (position - -1);
        double output = 0 + ((1.0 - 0) / (1 - -1)) * (position - -1);

        myTelemetry.addData("Ranged Output", output);
        //myTelemetry.update();
        linkMotor.setPosition(output);
    }

    public void linkPickPos(){
        linkMotor.setPosition(linkPos);
    }

    public void clawOpen(){
        clawMotor.setPosition(0.55);
    }
    public void clawClose(){
        clawMotor.setPosition(0);
    }

    public void dropOnePixel(){
        clawMotor.setPosition(0.8);
    }

    public void safetyMove(){

        if (pixelArm.getCurrentPosition() > pickPos-100) {

            pixelArm.setTargetPosition((int) (pickPos-100));

            pixelArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            pixelArm.setPower(0.4);
        }
    }

    public void incrementUp(){
        int pos = pixelArm.getCurrentPosition();
        pixelArm.setTargetPosition((int) (pos-10));

        pixelArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pixelArm.setPower(0.2);
    }

    public void incrementDown(){
        int pos = pixelArm.getCurrentPosition();
        pixelArm.setTargetPosition((int) (pos+10));

        pixelArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pixelArm.setPower(0.2);
    }

    public void overideSafety(){
        pixelArm.setTargetPosition((int) (pickPos-50));

        pixelArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pixelArm.setPower(0.2);

        linkMotor.setPosition(0.45);
    }

    public void dropInAuton() throws InterruptedException {

        pixelArm.setTargetPosition((int) (100));

        pixelArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pixelArm.setPower(0.2);

        sleep(2000);

        linkMotor.setPosition(0.146);

    }

    public void startPosInAuton(int target) throws InterruptedException {
        pixelArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        myTelemetry.addData("Target Pos ", target);
        controller.setPID(p, i, d);
        int armPos = pixelArm.getCurrentPosition();
        myTelemetry.addData("current ARM position", armPos);

        double pid = controller.calculate(armPos, target);
        double ff = Math.cos(Math.toRadians(target/TICKS_IN_DEGREE)) * f;
        double power = pid + ff;
        myTelemetry.addData("current power", power);

        pixelArm.setPower(power);
        myTelemetry.addData("ARM POS", pixelArm.getCurrentPosition());
        myTelemetry.update();
        sleep(2000);
        linkMotor.setPosition(0.375);
        clawClose();
    }

    public void autonPosMaintain(int target1) throws InterruptedException {
        pixelArm.setTargetPosition((int) (target1));

        pixelArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pixelArm.setPower(0.2);

        while (pixelArm.getCurrentPosition() != target1){
            myTelemetry.addData("Current Arm Pos: ", pixelArm.getCurrentPosition());
            sleep(1000);
        }

        double stallPower = 0.01;
        myTelemetry.addData("Target pos reached, setting stall power ", stallPower );

        pixelArm.setPower(stallPower);

    }

    public void holdPos(){
        double degreesInTicks = 360/537; //measure and change
        double angleOfArm = pixelArm.getCurrentPosition() * degreesInTicks;

        double totalAngle = 180.0; // Total angle range
        double totalPower = 0.2;   // Total power range

        // Calculate power based on the linear relationship between angle and power
        double power = (angleOfArm / totalAngle) * totalPower - (totalPower / 2);

        pixelArm.setPower(power);
    }

    // 120 degrees --> need to travel 1760 ticks
    public void encoderDrive(double speed, double turnAngle) {
        int newArmTarget;


        // Determine new target position, and pass to motor controller
        newArmTarget = pixelArm.getCurrentPosition() + (int) (turnAngle * COUNTS_PER_INCH);
        pixelArm.setTargetPosition(newArmTarget);

        // Turn On RUN_TO_POSITION
        pixelArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        runtime.reset();
        pixelArm.setPower(Math.abs(speed));

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

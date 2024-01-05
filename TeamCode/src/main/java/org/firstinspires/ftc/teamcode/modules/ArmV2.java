package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ArmV2 {

    public enum STATE {
        MOVING_L2_UP,
        MOVING_L2_DOWN,
        LINK2_IDEL
    };

    public enum EVENT {
        DRIVE_WITH_PIXEL_POS,
        DRIVE_WITHOUT_PIXEL_POS,
        ARM_ALLIGN,
        PIXEL_PICK_UP,
        BACK_DROP_AUTON,
        DROP_BACKDROP,
        STARTER_STACK_PICK_UP,
        DROP_LEFT_PIXEL,
        DROP_RIGHT_PIXEL,
        DROP_BOTH_PIXELS,
        CLAW_CLOSE,
        ARM_INCREMENT_UP,
        ARM_INCREMENT_DOWN,
        ARM_MOVE_UP,
        ARM_MOVE_DOWN,
        ARM_STOP,
        PID
    }
    EVENT currentEvent;

    public Link1 link1;
    public Link2 link2;
    public Claw claw;
    public double currentAngle;

    public double targetAngle;

    public double power=0.2;
    double l2targetAngle = 0;
    private double kp = 0.009; // Proportional gain
    private double ki = 0; // Integral gain
    private double kd = 0; // Derivative gain
    private double integral = 0;
    private double previousError = 0;

    public boolean PID_Trigger = false;

    Telemetry armTelemetry;
    public ArmV2(HardwareMap hardwareMap, Telemetry telemetry) {
        // Create an instance of the base class
        link1 = new Link1(hardwareMap.get(Servo.class, "linkLeft"),
                hardwareMap.get(Servo.class, "linkRight"));

        link2 = new Link2(hardwareMap.get(DcMotorEx.class, "armMotor"), hardwareMap.get(AnalogInput.class, "pot"));

        claw = new Claw(hardwareMap.get(Servo.class, "clawLeft"), hardwareMap.get(Servo.class, "clawRight"));
        armTelemetry = telemetry;
    }

    public void transition(EVENT event) {
        switch (event) {
            case DRIVE_WITH_PIXEL_POS:
                link2.updateTargetPos(30);
                claw.moveBothClaws(0.5);
                link1.moveLinks(0);
                break;
            case DROP_LEFT_PIXEL:
                claw.moveClaw(claw.clawServoL, 0);
                break;
            case DROP_RIGHT_PIXEL:
                claw.moveClaw(claw.clawServoR, 0);
            case BACK_DROP_AUTON:
                link2.updateTargetPos(150);
                link1.moveLinks(0.7);
                break;
            case DROP_BOTH_PIXELS:
                claw.moveBothClaws(0);
                break;
            case ARM_ALLIGN:
                link2.updateTargetPos(50);
                claw.moveBothClaws(0);
                link1.moveLinks(0);
                break;
            case STARTER_STACK_PICK_UP:
                link2.updateTargetPos(30);
                claw.moveBothClaws(0);
                link1.moveLinks(0);
                break;
            case DRIVE_WITHOUT_PIXEL_POS:
                link2.updateTargetPos(30);
                claw.moveBothClaws(0);
                link1.moveLinks(0);
                break;
            case PIXEL_PICK_UP:
                link2.updateTargetPos(15);
                link1.moveLinks(0);
                claw.moveBothClaws(0);
                break;
            case CLAW_CLOSE:
                claw.moveBothClaws(0.5);
                break;
            case DROP_BACKDROP:
                link2.updateTargetPos(120);
                link1.moveLinks(1);
                break;
            case ARM_INCREMENT_UP:
                link2.updateTargetPos(link2.getAngle()+5);
                break;
            case ARM_INCREMENT_DOWN:
                link2.updateTargetPos(link2.getAngle()-5);
                break;
            case ARM_MOVE_UP:
                link2.moveUp();
                break;
            case ARM_MOVE_DOWN:
                link2.moveDown();
                break;
            case ARM_STOP:
                link2.stopArm();
                break;
            case PID:
                PID_Trigger = !(PID_Trigger);

        }
    }



    public void update()
    {
        //link2.update();
        if (PID_Trigger) {
            link2.pid(targetAngle);
        }
        armTelemetry.addData("L2 target angle", targetAngle );
        armTelemetry.addData("L2 state", link2.L2State);
        armTelemetry.addData("L2 current angle", link2.getAngle());
        armTelemetry.update();
    }

    private class Link1 {
        public Servo l1LeftServo;
        public Servo l1RightServo;

        public Link1(Servo leftServo, Servo rightServo) {
            l1LeftServo = leftServo;
            l1RightServo = rightServo;
        }

        public void moveLinks(double pos){
            l1LeftServo.setPosition(pos);
            l1RightServo.setPosition(pos);
        }
    }

    private class Link2 {
        DcMotorEx link2ArmMotor;
        AnalogInput potmeter;
        public double l2targetAngle;



        public STATE L2State = STATE.LINK2_IDEL;

        public Link2(DcMotorEx armMotor, AnalogInput pot) {
            link2ArmMotor = armMotor;
            link2ArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            potmeter = pot;
        }

        public double getAngle(){
            double currentVoltage = potmeter.getVoltage();
            // output = output_start + ((output_end - output_start)/(input_end - input_start))*(input-input_start)
            double output = 0 + ((360 - 0)/(3.3 - 0))*(currentVoltage-0);
            // output = (270)/(3.3)*currentVoltage
           // currentAngle = output; //find correct equation
            return output;
        }

        //Drop for auto is 150
        //Drop for normal is 120
        // Changing from run to position to analog POT input
        public void moveUp() {
            link2ArmMotor.setPower(power*-1);
            L2State = STATE.MOVING_L2_UP;
        }
        public void moveDown() {
            link2ArmMotor.setPower(power);
            L2State = STATE.MOVING_L2_DOWN;
        }
        public void stopArm(){
            link2ArmMotor.setPower(0);
        }
        public void incrementUp(){
            int pos = link2ArmMotor.getCurrentPosition();
            link2ArmMotor.setTargetPosition((int) (pos-10));
            link2ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            link2ArmMotor.setPower(0.2);
        }

        public void incrementDown(){
            int pos = link2ArmMotor.getCurrentPosition();
            link2ArmMotor.setTargetPosition((int) (pos+10));
            link2ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            link2ArmMotor.setPower(0.2);
        }

        public void moveToAngle(double angle){
            l2targetAngle = angle;
            double armAngle = getAngle();
            if (armAngle < angle) {
                moveUp();
            } else if (armAngle > angle) {
                moveDown();
            }
        }

//        public void update() {
//            currentAngle = getAngle();
//            if (L2State == STATE.MOVING_L2_DOWN) {
//                if (currentAngle < l2targetAngle) {
//                    link2ArmMotor.setPower(0);
////                int pos = link2ArmMotor.getCurrentPosition();
////                link2ArmMotor.setTargetPosition((int) (pos+1));
////                link2ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////                link2ArmMotor.setPower(0.2);
//                }
//            }
//            if (L2State == STATE.MOVING_L2_UP) {
//                if (currentAngle > l2targetAngle) {
//                    link2ArmMotor.setPower(0);
//                }
//            }
//
//        }

        public void pid(double target){
            double angle = getAngle();
            double error = target - angle;

            // Proportional term
            double proportional = kp * error;

            // Integral term (with anti-windup)
            integral += ki * error;
            integral = Range.clip(integral, -1 / ki, 1 / ki);

            // Derivative term
            double derivative = kd * (error - previousError);

            // PID output
            double output = proportional + integral + derivative;

            // Apply the output to the motor
            link2ArmMotor.setPower(-output);

            // Update previous error
            previousError = error;

            // Check if the motor has reached the target position
            if (Math.abs(error) < 1) {
                link2ArmMotor.setPower(0); // Stop the motor
            }

        }
        public void updateTargetPos(double target){
            targetAngle = target;
        }
    }

    private class Claw {
        Servo clawServoL;
        Servo clawServoR;
        public Claw(Servo clawLeft, Servo clawRight) {
            clawServoL = clawLeft;
            clawServoR = clawRight;
            clawServoR.setDirection(Servo.Direction.REVERSE);
        }

        public void moveClaw(Servo servo, double pos){
            servo.setPosition(pos);
        }
        public void moveBothClaws(double pos){
            clawServoR.setPosition(pos);
            clawServoL.setPosition(pos);
        }
    }
}
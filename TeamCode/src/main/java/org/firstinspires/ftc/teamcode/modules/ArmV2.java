package org.firstinspires.ftc.teamcode.modules;

import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.robotcore.external.Telemetry;
@Config
public class ArmV2 {

    public static ArmV2 Link1;

    public enum STATE {
        MOVING_L2_UP,
        MOVING_L2_DOWN,
        LINK2_IDEL,
        ARM_IN_PICK,
        ARM_IN_DRIVE,
        ARM_IN_DROP,
        ARM_MOVE_PICK,
        ARM_MOVE_DRIVE,
        ARM_MOVE_DROP
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
        DROP_PURPLE,
        AUTON_POS,
        PID
    }
    EVENT currentEvent;

    public Link1 link1;
    public Link2 link2;
    public Claw claw;
    public double currentAngle;
    public double targetAngle;
    public boolean PID_Trigger = false;
    public boolean armReached = false;



    public static double kp = 0.007; // Proportional gain
    public static double ki = 0.0000; // Integral gain
    public static double kd = 0.0000; // Derivative gain

    public static double kf = 0.1;

    Telemetry armTelemetry;
    public ArmV2(HardwareMap hardwareMap, Telemetry telemetry) {
        // Create an instance of the base class
        link1 = new Link1(hardwareMap.get(Servo.class, "linkLeft"),
                hardwareMap.get(Servo.class, "linkRight"));

        link2 = new Link2(hardwareMap.get(DcMotorEx.class, "armMotor"), hardwareMap.get(AnalogInput.class, "pot"));

        claw = new Claw(hardwareMap.get(Servo.class, "clawLeft"), hardwareMap.get(Servo.class, "clawRight"));
        armTelemetry = telemetry;
        targetAngle = link2.getAngle();
    }

    public void transition(EVENT event){
        switch (event) {
            case DRIVE_WITH_PIXEL_POS:
                claw.close();
                link2.updateTargetPos(-5);
                link1.moveLinks(1);
                break;
            case DROP_LEFT_PIXEL:
                claw.openLeft();
                break;
            case DROP_RIGHT_PIXEL:
                claw.openRight();
                break;
            case BACK_DROP_AUTON:
                link2.updateTargetPos(150);
                link1.moveLinks(0.7);
                break;
            case DROP_BOTH_PIXELS:
                claw.open();
                break;
            case ARM_ALLIGN:
                link2.updateTargetPos(30);
                claw.open();
                link1.pickUp();
                break;
            case STARTER_STACK_PICK_UP:
                link2.updateTargetPos(7);
                claw.open();
                link1.pickUp();
                break;
            case DRIVE_WITHOUT_PIXEL_POS:
                claw.close();
                // link2.updateTargetPos(5);
                link1.moveLinks(1);
                break;
            case PIXEL_PICK_UP:
                link2.updateTargetPos(-3);
                //claw.open();
                link1.pickUp();
                break;
            case CLAW_CLOSE:
                claw.close();
                break;
            case DROP_BACKDROP:
                link2.updateTargetPos(130);
                link1.drop();
                break;
            case ARM_INCREMENT_UP:
                link2.updateTargetPos(targetAngle + 1);
                break;
            case ARM_INCREMENT_DOWN:
                link2.updateTargetPos(targetAngle - 1);
                break;
            case ARM_MOVE_UP:
                link2.moveUp();
                break;
            case ARM_MOVE_DOWN:
                link2.moveDown();
                break;
            case ARM_STOP:
                link1.moveLinks(1);
                break;
            case PID:
                link2.updateTargetPos(5);
                link1.moveLinks(1);
                break;
            case DROP_PURPLE:
                link2.updateTargetPos(30);
                link1.pickUp();
                //claw.openRight();
                break;
            case AUTON_POS:
                //link2.updateTargetPos(40);
                link1.pickUp();

        }
    }

    public void update()
    {
        link2.update();
        armTelemetry.addData("L2 target angle", targetAngle );
        armTelemetry.addData("L2 state", link2.L2State);
        armTelemetry.addData("L2 current angle", link2.getAngle());
        armTelemetry.update();
    }


    public class Link1 {
        public Servo l1LeftServo;
        public Servo l1RightServo;

        public Link1(Servo leftServo, Servo rightServo) {
            l1LeftServo = leftServo;
            l1RightServo = rightServo;
        }

        public void moveLinks(double pos){
            l1LeftServo.setPosition(pos-0.01);
            l1RightServo.setPosition(pos);
        }

        public void pickUp(){
            link1.moveLinks(0.07);
        }
        public void drop(){
            link1.moveLinks(0.85);
        }

    }
    @Config
    public class Link2 {
        DcMotorEx link2ArmMotor;
        AnalogInput potmeter;
        public double l2targetAngle;

        public double power=0.2;
        double proportional;
        double derivative;
        private double integral = 0;
        private double previousError = 0;

        private final double ARM_ERROR_OFFSET = 5;


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
            link2ArmMotor.setTargetPosition((int) (link2.getAngle() + 40));
        }

        public void incrementDown(){
            link2ArmMotor.setTargetPosition((int) (link2.getAngle() - 40));

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

        public void update() {
            link2.link2ArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            link2.pid(targetAngle);
            currentAngle = getAngle();
            if (L2State == STATE.MOVING_L2_DOWN) {
                if (currentAngle < l2targetAngle) {
                    link2ArmMotor.setPower(0);
                    L2State = STATE.LINK2_IDEL;
                }

            }
            if (L2State == STATE.MOVING_L2_UP) {
                if (currentAngle > l2targetAngle) {
                    link2ArmMotor.setPower(0);
                    L2State = STATE.LINK2_IDEL;
                }
            }

        }

        public void pid(double target){

//            if ((link2.getAngle()+5 == target) || (link2.getAngle()-5 == target)){
//                armTelemetry.addLine("KP INCREASED");
//                kp = 0.1;
//            }

            double angle = getAngle();
            double error = target - angle;

            // Proportional term
            proportional = kp * error;

            // Integral term (with anti-windup)
            integral += ki * error;
            integral = Range.clip(integral, -1 / ki, 1 / ki);

            // Derivative term
            derivative = kd * (error - previousError);

            // PID output
            double output = proportional + integral + derivative; //+(Math.cos(link2.getAngle() - 21) * kf) ;


            // Apply the output to the motor
            link2ArmMotor.setPower(-output );

            // Update previous error
            previousError = error;

            if (Math.abs(error) < 5) {
                armReached = true;

            }else{
                armReached = false;
            }

        }


        public void updateTargetPos(double target){
            targetAngle = target;
        }
    }

    public class Claw {
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

        public void close(){
            claw.closeLeft();
            claw.closeRight();
        }
        public void open(){
            claw.openLeft();
            claw.openRight();
        }
        public void moveBothClaws(double pos){
            clawServoR.setPosition(pos);
            clawServoL.setPosition(pos);
        }

        public void openLeft(){
            clawServoL.setPosition(0.01);
        }
        public void openRight(){
            clawServoR.setPosition(0);
        }

        public void closeLeft(){
            clawServoL.setPosition(0.47);
        }
        public void closeRight(){
            clawServoR.setPosition(0.4);
        }
    }
}
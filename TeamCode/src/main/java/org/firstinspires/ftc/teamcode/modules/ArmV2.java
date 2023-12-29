package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ArmV2 {
    public enum EVENT {
        DRIVE_WITH_PIXEL_POS,
        DRIVE_WITHOUT_PIXEL_POS,
        ARM_ALLIGN,
        PIXEL_PICK_UP,
        BACK_DROP_AUTON,
        DROP_BACKDROP,
        STARTER_STACK_PICK_UP,
        DROP_BOTTOM_PIXEL,
        DROP_BOTH_PIXELS,
        CLAW_CLOSE,
        ARM_INCREMENT_UP,
        ARM_INCREMENT_DOWN,
        ARM_MOVE_UP,
        ARM_MOVE_DOWN,
        ARM_STOP,
    }
    EVENT currentEvent;

    public Link1 link1;
    public Link2 link2;
    public Claw claw;
    public double currentAngle;


    public ArmV2(HardwareMap hardwareMap, Telemetry telemetry) {
        // Create an instance of the base class
        link1 = new Link1(hardwareMap.get(Servo.class, "linkLeft"),
                hardwareMap.get(Servo.class, "linkRight"));

        link2 = new Link2(hardwareMap.get(DcMotorEx.class, "armMotor"), hardwareMap.get(AnalogInput.class, "pot"));

        claw = new Claw(hardwareMap.get(Servo.class, "clawMotor"));
    }

    public void transition(EVENT event) {
        switch (event) {
            case DRIVE_WITH_PIXEL_POS:
                link2.moveToAngle(30);
                claw.close();
                link1.moveLinks(1);
            case DROP_BOTTOM_PIXEL:
                claw.open1();
            case BACK_DROP_AUTON:
                link2.moveToAngle(150);
                link1.moveLinks(0.3);
            case DROP_BOTH_PIXELS:
                claw.open2();
            case ARM_ALLIGN:
                link2.moveToAngle(50);
                claw.open2();
                link1.moveLinks(1);
            case STARTER_STACK_PICK_UP:
                link2.moveToAngle(30);
                claw.open2();
                link1.moveLinks(1);
            case DRIVE_WITHOUT_PIXEL_POS:
                link2.moveToAngle(30);
                claw.open2();
                link1.moveLinks(1);
            case PIXEL_PICK_UP:
                link2.moveToAngle(15);
                link1.moveLinks(1);
                claw.open2();
            case CLAW_CLOSE:
                claw.close();
            case DROP_BACKDROP:
                link2.moveToAngle(120);
                link1.moveLinks(0);
            case ARM_INCREMENT_UP:
                link2.incrementUp();
            case ARM_INCREMENT_DOWN:
                link2.incrementDown();
            case ARM_MOVE_UP:
                link2.moveUp();
            case ARM_MOVE_DOWN:
                link2.moveDown();
            case ARM_STOP:
                link2.stopArm();

        }
    }

    private class Link1 {
        public Servo l1LeftServo;
        public Servo l1RightServo;

        public Link1(Servo leftServo, Servo rightServo) {
            l1LeftServo = leftServo;
            l1RightServo = rightServo;
            l1LeftServo.setDirection(Servo.Direction.REVERSE);
        }

        public void moveLinks(double pos){
            l1LeftServo.setPosition(pos);
            l1RightServo.setPosition(pos);
        }
    }

    private class Link2 {
        DcMotorEx link2ArmMotor;
        AnalogInput potmeter;

        public Link2(DcMotorEx armMotor, AnalogInput pot) {
            link2ArmMotor = armMotor;
            link2ArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            potmeter = pot;
        }

        public double getAngle(){
            double currentVoltage = potmeter.getVoltage();
            double currentAngle = currentVoltage * 81.8; //find correct equation

            return currentAngle;
        }
        //Drop for auto is 150
        //Drop for normal is 120
        // Changing from run to position to analog POT input
        public void moveUp() {
            link2ArmMotor.setPower(0.4);
        }
        public void moveDown() {
            link2ArmMotor.setPower(-0.4);
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
            double armAngle = getAngle();
            while(currentAngle!=angle) {
                armAngle = getAngle();
                if (armAngle < angle) {
                    link2ArmMotor.setPower(0.1);
                } else if (armAngle > angle) {
                    link2ArmMotor.setPower(-0.1);
                }

                if (currentAngle == angle) {
                    link2ArmMotor.setPower(0);
                    int pos = link2ArmMotor.getCurrentPosition();
                    link2ArmMotor.setTargetPosition((int) (pos+1));

                    link2ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    link2ArmMotor.setPower(0.2);
                }
            }
        }

    }

    private class Claw {
        Servo clawServo;
        public Claw(Servo cservo) {clawServo = cservo;}
        public void open1() {
            clawServo.setPosition(0.5);
        } //Drops bottom pixel
        public void open2() {
            clawServo.setPosition(0.2);
        } //Drops both pixel
        public void close() {
            clawServo.setPosition(0.9);
        }
    }
}
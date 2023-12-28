package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ArmV2 {
    public enum EVENT {
        ARM_PICK_UP_POSITION,
        PIXEL_PICK_UP,
        PIXEL_HOLD,
        BACK_DROP_1,
        BACK_DROP_2,
        STARTER_STACK_PICK_UP,
        PURPLE_PIXEL_DROP,
        YELLOW_PIXEL_DROP,
    }
    EVENT currentEvent;

        public Link1 link1;
        public Link2 link2;
        public Claw claw;

        public ArmV2(HardwareMap hardwareMap, Telemetry telemetry) {
            // Create an instance of the base class
            Link1 link1 = new Link1(hardwareMap.get(Servo.class, "linkLeft"),
                                    hardwareMap.get(Servo.class, "linkRight"));
            Link2 link2 = new Link2(hardwareMap.get(DcMotorEx.class, "armMotor"));
            Claw claw = new Claw(hardwareMap.get(Servo.class, "clawMotor"));
        }

        public void transition(EVENT event) {
            switch (event) {
                case PIXEL_PICK_UP:
                    link1.moveDown();
                    link2.moveDown2();
                    claw.open2();
                case ARM_PICK_UP_POSITION:
                    link1.moveDown();
                    link2.moveDown1();
                    claw.close();
                case PIXEL_HOLD:
                    link1.moveDown();
                    link2.moveDown1();
                    claw.close();
                case BACK_DROP_1:
                    link1.moveUp();
                    link2.moveUp();;
                case BACK_DROP_2:
                    link1.moveUp();
                    link2.moveUp();
                    claw.open2();
                case STARTER_STACK_PICK_UP:
                    link1.moveDown();
                    link2.moveDown2();
                    claw.open1();
                case PURPLE_PIXEL_DROP:
                    link1.moveDown();
                    link2.moveDown1();
                    claw.open1();
                case YELLOW_PIXEL_DROP:
                    link1.moveUp();
                    link2.moveUp();
                    claw.open2();
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

    public void moveUp() {
        l1LeftServo.setPosition(0);
        l1RightServo.setPosition(0);
    }

    public void moveDown() {
        l1LeftServo.setPosition(1);
        l1RightServo.setPosition(1);
    }
}

private class Link2 {
    DcMotorEx link2ArmMotor;

    public Link2(DcMotorEx armMotor) {  link2ArmMotor = armMotor; }
    // Changing from run to position to analog POT input
    public void moveUp() {
        link2ArmMotor.setPower(0.6);
    }
    public void moveDown1() {
        link2ArmMotor.setPower(-0.4);
    }
    public void moveDown2() {
        link2ArmMotor.setPower(-0.2);
    }
}

private class Claw {
    Servo clawServo;
    public Claw(Servo cservo) { clawServo = cservo;     }
    public void open1() {
        clawServo.setPosition(0.7);
    }
    public void open2() {
        clawServo.setPosition(0.9);
    }
    public void close() {
        clawServo.setPosition(0.2);
    }
  }
}
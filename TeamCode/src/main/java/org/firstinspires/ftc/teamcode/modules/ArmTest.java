package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ArmTest {

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

    Servo clawMotor;
    // Base class
    class Link1 {
        Telemetry l1Telemetry;
        HardwareMap l1hwMap;
        Servo l1LeftServo;
        Servo l1RightServo;

        public Link1(HardwareMap hardwareMap, Telemetry telemetry) {
            l1Telemetry = telemetry;
            l1hwMap = hardwareMap;
            l1LeftServo = l1hwMap.get(Servo.class, "linkLeft");
            l1LeftServo.setDirection(Servo.Direction.REVERSE);
            l1RightServo = l1hwMap.get(Servo.class, "linkRight");
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

    class Link2 {
        Telemetry l2Telemetry;
        HardwareMap l2hwMap;
        DcMotorEx link2ArmMotor;

        public Link2(HardwareMap hardwareMap, Telemetry telemetry) {
            l2Telemetry = telemetry;
            l2hwMap = hardwareMap;
            link2ArmMotor = l2hwMap.get(DcMotorEx.class, "link2ArmMotor");
        }

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

    class Claw {
        private String robotName;
        Telemetry cTelemetry;
        HardwareMap chwMap;
        Servo clawServo;

        public Claw(HardwareMap hardwareMap, Telemetry telemetry) {
            cTelemetry = telemetry;
            chwMap = hardwareMap;
            clawServo = chwMap.get(Servo.class, "clawMotor");
        }

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

    public class Arm {
        Link1 link1;
        Link2 link2;
        Claw claw;

        public Arm(HardwareMap hardwareMap, Telemetry telemetry) {
            // Create an instance of the base class
            Link1 link1 = new Link1(hardwareMap, telemetry);
            Link2 link2 = new Link2(hardwareMap, telemetry);
            Claw claw = new Claw(hardwareMap, telemetry);
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
    }
}
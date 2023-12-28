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

        private String robotName;

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
        private String robotName;
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
            System.out.println(robotName + " is starting.");
        }

        public void moveDown1() {
            link2ArmMotor.setPower(-0.4);
            System.out.println(robotName + " is moving forward.");
        }

        public void moveDown2() {
            link2ArmMotor.setPower(-0.2);
            System.out.println(robotName + " has stopped.");
        }
    }

    class Claw {
        private String robotName;
        Telemetry l2Telemetry;
        HardwareMap l2hwMap;
        Servo clawMotor;

        public Claw(String name) {
            this.robotName = name;
        }

        public void open1() {
            clawMotor.setPosition(0.7);
            System.out.println(robotName + " is starting.");
        }
        public void open2() {
            clawMotor.setPosition(0.9);
            System.out.println(robotName + " is moving forward.");
        }

        public void close() {
            clawMotor.setPosition(0.2);
            System.out.println(robotName + " has stopped.");
        }
    }

    public class Arm {
        public Arm(HardwareMap hardwareMap, Telemetry telemetry) {
            // Create an instance of the base class
            Link1 link1 = new Link1(hardwareMap,telemetry);
            Link2 link2 = new Link2(hardwareMap, telemetry);
            Claw claw = new Claw(hardwareMap, telemetry);

            if (currentEvent == EVENT.PIXEL_PICK_UP) {
                link1.moveDown();
                link2.moveDown2();
                claw.open2();
            }

            if (currentEvent == EVENT.ARM_PICK_UP_POSITION) {
                link1.moveDown();
                link2.moveDown1();
                claw.close();
            }
            if (currentEvent == EVENT.PIXEL_HOLD) {
                link1.moveDown();
                link2.moveDown1();
                claw.close();
            }
            if (currentEvent == EVENT.BACK_DROP_1) {
                link1.moveUp();
                link2.moveUp();
                claw.open1();
            }
            if (currentEvent == EVENT.BACK_DROP_2) {
                link1.moveUp();
                link2.moveUp();
                claw.open2();
            }
            if (currentEvent == EVENT.STARTER_STACK_PICK_UP) {
                link1.moveDown();
                link2.moveDown2();
                claw.open1();
            }
            if (currentEvent == EVENT.PURPLE_PIXEL_DROP) {
                link1.moveDown();
                link2.moveDown1();
                claw.open1();
            }
            if (currentEvent == EVENT.YELLOW_PIXEL_DROP) {
                link1.moveUp();
                link2.moveUp();
                claw.open2();
            }

        }

    }
}
package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

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
    Servo link1l;
    Servo link1r;
    DcMotorEx link2ArmMotor;
    Servo clawMotor;
        // Base class
    class Link1 {
        private String robotName;

        public Link1(String name) {
            this.robotName = name;
        }

        public void moveUp() {
            System.out.println(robotName + " is starting.");
        }

        public void moveDown() {
            System.out.println(robotName + " is moving forward.");
        }

        public void stop() {
            System.out.println(robotName + " has stopped.");
        }
    }

    class Link2 {
        private String robotName;

        public Link2(String name) {
            this.robotName = name;
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
        public Arm() {
            // Create an instance of the base class
            Link1 link1 = new Link1("RegularRobot");
            Link2 link2 = new Link2("RegularRobot");
            Claw claw = new Claw("RegularRobot");

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

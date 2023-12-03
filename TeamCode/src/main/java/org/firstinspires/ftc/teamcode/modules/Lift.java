package org.firstinspires.ftc.teamcode.modules;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Lift {

    DcMotor lift;


    public enum STATE {
        LIFT_OPENING_UP,
        LIFT_GRABBING_TRUSS,
        LIFT_RETRACTED,

    }

    public enum EVENT {
        ROBOT_UNDER_TRUSS,
        LIFT_GRABBED_TRUSS,
        ROBOT_HANGING,
    }

    Lift.STATE currentState;

    public Lift.STATE getState() {
        return currentState;
    }

    public Lift(HardwareMap hardwareMap, Telemetry telemetry) {
        lift = hardwareMap.get(DcMotor.class, "lift");
    }

    public void transition(EVENT event) {
        switch (event) {
            case ROBOT_UNDER_TRUSS:
                currentState = Lift.STATE.LIFT_OPENING_UP;
                break;
            case LIFT_GRABBED_TRUSS:
                currentState = Lift.STATE.LIFT_GRABBING_TRUSS;
                break;
            case ROBOT_HANGING:
                currentState = Lift.STATE.LIFT_RETRACTED;
                break;
        }
    }

    public void update() {
        switch (currentState) {
            case LIFT_GRABBING_TRUSS:
                lift.setPower(0.2);
                break;
            case LIFT_RETRACTED:
                lift.setPower(-0.2);
                break;
        }
    }
}
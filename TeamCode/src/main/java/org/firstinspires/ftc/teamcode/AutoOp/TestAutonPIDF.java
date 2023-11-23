package org.firstinspires.ftc.teamcode.AutoOp;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.modules.Arm;

@Autonomous(name = "Arm PIDF Test in Auton")
public class TestAutonPIDF extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException {
        Arm pixelArm = new Arm(hardwareMap, telemetry);

        pixelArm.resetEncoder();
        pixelArm.startPosInAuton(0);
    }
}

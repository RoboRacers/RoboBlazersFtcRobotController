package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

@TeleOp
public class POT_test extends LinearOpMode {

    AnalogInput pot;

    @Override
    public void runOpMode() throws InterruptedException {

        pot = hardwareMap.get(AnalogInput.class, "pot");


        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("voltage", pot.getVoltage());
            telemetry.addData("angle", pot.getVoltage()*81.8);

            double output = 0 + ((360 - 0) / (3.3 - 0)) * (pot.getVoltage() - 0);

            telemetry.addData("correct output: ", output);

            telemetry.update();

        }
    }
}

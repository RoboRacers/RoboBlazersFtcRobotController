package org.firstinspires.ftc.teamcode.modules.util;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;
public class YellowPixelDetector {
    private final int READ_PERIOD = 1;


    public double id1Center;
    public double id2Center;
    public double id3Center;
    public double pixelCenter;
    public String placement;

    public Telemetry telemetry;
    HuskyLens huskyLens;


    public YellowPixelDetector(HardwareMap hardwareMap, Telemetry myTelemetry){
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        telemetry = myTelemetry;
    }

    public String getYellowPlacement(){

        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

        for (int x = 0; x <= 49; x++) {
            HuskyLens.Block[] tagBlocks = huskyLens.blocks();
            telemetry.addData("Block count", tagBlocks.length);
            for (int i = 0; i < tagBlocks.length; i++) {

                telemetry.addData("Tag", tagBlocks[i].toString());
                telemetry.addData("X center: ", tagBlocks[i].x);
                telemetry.addData("Y center: ", tagBlocks[i].y);
                //telemetry.update();

                if (tagBlocks[i].id == 1) {
                    id1Center += tagBlocks[i].x;
                }
                if (tagBlocks[i].id == 2) {
                    id2Center += tagBlocks[i].x;
                }
                if (tagBlocks[i].id == 3) {
                    id3Center += tagBlocks[i].x;
                }

                //telemetry.update();
            }
        }
        id1Center /= 50;
        id2Center /= 50;
        id3Center /= 50;

        double range2 = id1Center + ((Math.abs(id1Center - id2Center)) / 2);
        double range1 = id1Center - ((Math.abs(id1Center - id2Center)) / 2);
        double range3 = id2Center + ((Math.abs(id2Center - id3Center)) / 2);
        double range4 = id3Center + ((Math.abs(id2Center - id3Center)) / 2);

        for (int y = 0; y < 50; y++) {

            huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);

            HuskyLens.Block[] pixelBlocks = huskyLens.blocks();
            for (int i = 0; i < pixelBlocks.length; i++) {

                telemetry.addData("Pixel", pixelBlocks[i].toString());
                telemetry.addData("Pixel X center: ", pixelBlocks[i].x);
                telemetry.addData("Pixel Y center: ", pixelBlocks[i].y);
                pixelCenter = pixelBlocks[i].x;
                //telemetry.update();


                if (pixelCenter > range1 && pixelCenter < range2) {
                    placement = "LEFT";
                } else if (pixelCenter > range2 && pixelCenter < range3) {
                    placement = "CENTER";
                } else if (pixelCenter > range3 && pixelCenter < range4) {
                    placement = "RIGHT";
                } else {
                    placement = "NA";
                }

                telemetry.addData("Place:", placement);
                telemetry.update();
                //break;
            }
        }
        return placement;
    }
}

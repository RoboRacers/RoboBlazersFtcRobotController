package org.firstinspires.ftc.teamcode.AutoOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class TeamPropDetectionPipeline {
    TeamPropPipeline teamPropDetectionPipeline;
    Telemetry telemetry;

    public TeamPropDetectionPipeline(OpenCvCamera camera, Telemetry telemetry) {
        this.telemetry = telemetry;
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                teamPropDetectionPipeline = new TeamPropPipeline();
                camera.setPipeline(teamPropDetectionPipeline);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

        while (teamPropDetectionPipeline == null) {
        }
    }

    public String getDirection() {
        return teamPropDetectionPipeline.getDirection();
    }


    static class TeamPropPipeline extends OpenCvPipeline {

//        Mat gray = new Mat();
//        Mat shadowMask = new Mat();
//        Mat labImage = new Mat();
//        List<Mat> labChannels = new ArrayList<>();
//        Mat noShadow = new Mat();
        Mat firstGray = new Mat();
        Mat binaryImg = new Mat();
        Mat blackCountImg = new Mat();

        int frameCount = 1;


        private String direction = "";

        @Override
        public Mat processFrame(Mat frame) {

            if (frameCount == 4) {
//                Imgproc.cvtColor(frame, gray, Imgproc.COLOR_BGR2GRAY);
//
//                Imgproc.medianBlur(gray, gray, 15);
//
//                // Apply adaptive thresholding to identify shadows
//                Imgproc.adaptiveThreshold(gray, shadowMask, 0, Imgproc.ADAPTIVE_THRESH_MEAN_C, Imgproc.THRESH_BINARY, 11, 10);
//
//                // Invert the shadow mask
//                Core.bitwise_not(shadowMask, shadowMask);
//
//                // Convert the original image to LAB color space
//                Imgproc.cvtColor(frame, labImage, Imgproc.COLOR_BGR2Lab);
//
//                // Split LAB image into L, A, and B channels
//                Core.split(labImage, labChannels);
//
//                // Replace L channel with the shadow-masked L channel
//                labChannels.set(0, shadowMask);
//
//                // Merge LAB channels back into one image
//                Core.merge(labChannels, labImage);
//
//                // Convert LAB image back to BGR
//                Imgproc.cvtColor(labImage, noShadow, Imgproc.COLOR_Lab2BGR);
//
//
//                firstGray = noShadow.clone();

                Imgproc.cvtColor(frame, firstGray, Imgproc.COLOR_BGR2GRAY);

                Imgproc.threshold(firstGray, binaryImg, 0, 255, Imgproc.THRESH_BINARY + Imgproc.THRESH_OTSU);


                blackCountImg = binaryImg.clone();

                int leftBlackPixelCount = 0;

                int leftImageWidth = blackCountImg.cols() / 3;

                for (int row = 0; row < blackCountImg.rows(); row++) {
                    for (int col = 0; col < leftImageWidth; col++) {
                        double pixelValue = blackCountImg.get(row, col)[0];
                        if (pixelValue == 0) {
                            leftBlackPixelCount++;
                        }
                    }
                }


                int centerBlackPixelCount = 0;

                int centerImageWidth = 2 * (blackCountImg.cols() / 3);

                for (int row = 0; row < blackCountImg.rows(); row++) {
                    for (int col = leftImageWidth; col < centerImageWidth; col++) {
                        double pixelValue = blackCountImg.get(row, col)[0];
                        if (pixelValue == 0) {
                            centerBlackPixelCount++;
                        }
                    }
                }


                int rightBlackPixelCount = 0;

                int rightImageWidth = blackCountImg.cols();

                for (int row = 0; row < blackCountImg.rows(); row++) {
                    for (int col = centerImageWidth; col < rightImageWidth; col++) {
                        double pixelValue = blackCountImg.get(row, col)[0];
                        if (pixelValue == 0) {
                            rightBlackPixelCount++;
                        }
                    }
                }


                if (leftBlackPixelCount > centerBlackPixelCount && leftBlackPixelCount > rightBlackPixelCount) {
                    direction = "left";
                } else if (rightBlackPixelCount > leftBlackPixelCount && rightBlackPixelCount > centerBlackPixelCount) {
                    direction = "right";
                } else {
                    direction = "center";
                }
                frameCount = 1;
                return frame;
            } else if (frameCount != 4) {
                frameCount++;
            }
            return frame;
        }


        public String getDirection() {
            return direction;
        }

    }
}
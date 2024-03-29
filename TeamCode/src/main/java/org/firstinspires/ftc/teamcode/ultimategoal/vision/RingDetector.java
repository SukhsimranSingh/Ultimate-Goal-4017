package org.firstinspires.ftc.teamcode.ultimategoal.vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

@TeleOp
public class RingDetector extends LinearOpMode
{
    OpenCvCamera webcam;
    StackDeterminationPipeline stackDetector;
    DistanceFromRingTracker distanceDetector;
    OpenCvPipeline pipeline;
    enum Mode {
        STACK,
        DISTANCE
    }
    Mode currentMode = Mode.DISTANCE;


    @Override
    public void runOpMode()
    {
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"));
        FtcDashboard.getInstance().startCameraStream(webcam, 0);
        stackDetector = new StackDeterminationPipeline();
        distanceDetector = new DistanceFromRingTracker(
                0.5
        );
        pipeline = distanceDetector;
        webcam.setPipeline(pipeline);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320,240,  OpenCvCameraRotation.UPRIGHT);
            }
        });


        telemetry = FtcDashboard.getInstance().getTelemetry();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            switch (currentMode){
                case DISTANCE:
                    telemetry.addData("FPS", webcam.getFps());
                    telemetry.addData("Pipeline (ms)", webcam.getPipelineTimeMs());
                    telemetry.addData(
                            "Total Frame time (ms)",
                            webcam.getTotalFrameTimeMs()
                    );
                    telemetry.addData(
                            "W, H (px)",
                            distanceDetector.bounds.size.width +
                                    ", " +
                                    distanceDetector.bounds.size.height
                    );
                    telemetry.addData("distance (in)", distanceDetector.computeDistance());
                    telemetry.addData("focalLength", distanceDetector.computefocalLength());
                    telemetry.update();
                    if (gamepad1.a) {
                        pipeline = stackDetector;
                        currentMode = Mode.DISTANCE;
                    }
                    break;
                case STACK:
                    telemetry.addData("Analysis", stackDetector.getAnalysis());
                    telemetry.addData("Position", stackDetector.position);
                    telemetry.update();
                    if (gamepad1.b) {
                        pipeline = distanceDetector;
                        currentMode = Mode.DISTANCE;
                    }
                    break;

            }
            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }
    }

    public static class StackDeterminationPipeline extends OpenCvPipeline
    {
        /*
         * An enum to define the ring position
         */
        public enum RingPosition
        {
            FOUR,
            ONE,
            NONE
        }

        /*
         * Some color constants
         */
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        /*
         * The core values which define the location and size of the sample regions
         */
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(0,0);

        static final int REGION_WIDTH = 320;
        static final int REGION_HEIGHT = 240;

        final int FOUR_RING_THRESHOLD = 140;
        final int ONE_RING_THRESHOLD = 130;

        /*
         * Points which actually define the sample region rectangles, derived from above values
         *
         * Example of how points A and B work to define a rectangle
         *
         *   ------------------------------------
         *   | (0,0) Point A                    |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                  Point B (70,50) |
         *   ------------------------------------
         *
         */

        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        /*
         * Working variables
         */
        Mat region1_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1;

        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile RingPosition position = RingPosition.FOUR;

        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         */
        void inputToCb(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);
        }

        @Override
        public void init(Mat firstFrame)
        {
            inputToCb(firstFrame);

            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
        }

        @Override
        public Mat processFrame(Mat input)
        {
            inputToCb(input);

            avg1 = (int) Core.mean(region1_Cb).val[0];

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            position = RingPosition.FOUR; // Record our analysis
            if(avg1 > FOUR_RING_THRESHOLD){
                position = RingPosition.FOUR;

            }else if (avg1 > ONE_RING_THRESHOLD){
                position = RingPosition.ONE;
            }else{
                position = RingPosition.NONE;
            }

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill

            return input;
        }

        public int getAnalysis()
        {
            return avg1;
        }

        public void getFocalLength(double realDistance){

        }
    }


}
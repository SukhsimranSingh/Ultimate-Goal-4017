package org.firstinspires.ftc.teamcode.ultimategoal.Qualifier.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.advanced.PoseStorage;
import org.firstinspires.ftc.teamcode.ultimategoal.Qualifier.util.RPMTool;
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

/**
 * Example opmode demonstrating how to hand-off the pose from your autonomous opmode to your teleop
 * by passing the data through a static class.
 * <p>
 * This is required if you wish to read the pose from odometry in teleop and you run an autonomous
 * sequence prior. Without passing the data between each other, teleop isn't completely sure where
 * it starts.
 * <p>
 * This example runs the same paths used in the SplineTest tuning opmode. After the trajectory
 * following concludes, it simply sets the static value, `PoseStorage.currentPose`, to the latest
 * localizer reading.
 * However, this method is not foolproof. The most immediate problem is that the pose will not be
 * written to the static field if the opmode is stopped prematurely. To work around this issue, you
 * need to continually write the pose to the static field in an async trajectory follower. A simple
 * example of async trajectory following can be found at
 * https://www.learnroadrunner.com/advanced.html#async-following
 * A more advanced example of async following can be found in the AsyncFollowingFSM.java class.
 * <p>
 * The other edge-case issue you may want to cover is saving the pose value to disk by writing it
 * to a file in the event of an app crash. This way, the pose can be retrieved and set even if
 * something disastrous occurs. Such a sample has not been included.
 */
@Autonomous(group = "advanced")
public class AutoTest extends LinearOpMode {
    private Servo trigger;
    private Servo grabber;
    private DcMotorEx arm;
    private ElapsedTime runtime = new ElapsedTime();
    public static final double GRABBER_OPEN       =  0.1;
    public static final double GRABBER_CLOSED       =  0.9;
    OpenCvCamera webcam;
    StackDeterminationPipeline pipeline;


    @Override
    public void runOpMode() throws InterruptedException {
        // Declare your drive class
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        DcMotorEx launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        arm    = hardwareMap.get(DcMotorEx.class, "arm");

        launcher.setDirection(DcMotorSimple.Direction.FORWARD);
        RPMTool rpm = new RPMTool(launcher, 28);
        grabber  = hardwareMap.servo.get("grabber");
        trigger = hardwareMap.servo.get("trigger");

        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"));
        FtcDashboard.getInstance().startCameraStream(webcam, 0);
        pipeline = new StackDeterminationPipeline();
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
                webcam.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }
        });


        // Set the pose estimate to where you know the bot will start in autonomous
        // Refer to https://www.learnroadrunner.com/trajectories.html#coordinate-system for a map
        // of the field
        // This example sets the bot at x: 10, y: 15, and facing 90 degrees (turned counter-clockwise)
        Pose2d startPose = new Pose2d(-63, -24, Math.toRadians(0));

        drive.setPoseEstimate(startPose);
        grabber.setPosition(GRABBER_CLOSED);

        waitForStart();

        if (isStopRequested()) return;
        // Example spline path from SplineTest.java
        // Make sure the start pose matches with the localizer's start pose
        Trajectory zeroRingA = drive.trajectoryBuilder(startPose)//to launch line
                .lineTo(new Vector2d(-12,-24))
//                .addTemporalMarker(.5, () -> {
//                    rpm.setRPM(3000);
//                })
                .build();
        Trajectory zeroRingB = drive.trajectoryBuilder(zeroRingA.end())//launch rings
                .strafeTo(new Vector2d(-3, -16))
                .build();
        Trajectory zeroRingC = drive.trajectoryBuilder(zeroRingB.end())//launch rings
                .strafeTo(new Vector2d(-3, -8))
                .build();
        Trajectory zeroRingD = drive.trajectoryBuilder(zeroRingC.end())//launch rings
                .strafeTo(new Vector2d(-3, -2))
                .build();

        Trajectory zeroRingE = drive.trajectoryBuilder(zeroRingD.end())
                .lineToSplineHeading(new Pose2d(0,-44, Math.toRadians(180))) //drop wobble goal
                .build();

        Trajectory zeroRingF = drive.trajectoryBuilder(zeroRingE.end())//going to second
                .lineTo(new Vector2d(6, -40))
                .build();

        Trajectory zeroRingG = drive.trajectoryBuilder(zeroRingF.end())//lining up
                .strafeTo(new Vector2d(6,-36))
                .build();

        Trajectory zeroRingH = drive.trajectoryBuilder(zeroRingG.end())// second wobble
                .lineToConstantHeading(new Vector2d(-48, -35))
                .build();

        Trajectory zeroRingI = drive.trajectoryBuilder(zeroRingH.end()) //drop wobble and park
                .lineTo(new Vector2d(12, -36))
                .build();

        Trajectory oneRingA = drive.trajectoryBuilder(startPose)//to launch line
                .lineTo(new Vector2d(-12,-24))
                .build();
        Trajectory oneRingB = drive.trajectoryBuilder(oneRingA.end())//launch rings
                .strafeTo(new Vector2d(-3, -16))
                .build();
        Trajectory oneRingC = drive.trajectoryBuilder(oneRingB.end())//launch rings
                .strafeTo(new Vector2d(-3, -8))
                .build();
        Trajectory oneRingD = drive.trajectoryBuilder(oneRingC.end())//launch rings
                .strafeTo(new Vector2d(-3, -2))
                .build();
        Trajectory oneRingE = drive.trajectoryBuilder(oneRingD.end())
                .lineToSplineHeading(new Pose2d(22,-32, Math.toRadians(-90))) //drop wobble goal
                .build();
        Trajectory oneRingF = drive.trajectoryBuilder(oneRingE.end())//ring stack
                .lineToSplineHeading(new Pose2d(-28, -24, Math.toRadians(180)))
                .build();
        Trajectory oneRingG = drive.trajectoryBuilder(oneRingF.end())//
                .lineToConstantHeading(new Vector2d(-41, -37))
                .build();
        Trajectory oneRingH = drive.trajectoryBuilder(oneRingG.end())// second wobble
                .lineToConstantHeading(new Vector2d(-46, -37))
                .build();
        Trajectory oneRingI = drive.trajectoryBuilder(oneRingH.end()) //drop wobble
                .lineToSplineHeading(new Pose2d(16, -40, Math.toRadians(-90)))
                .build();
        Trajectory oneRingJ = drive.trajectoryBuilder(oneRingI.end()) //park
                .lineTo(new Vector2d(12, -40))
                .build();

        Trajectory fourRingA = drive.trajectoryBuilder(startPose)//to launch line
                .lineTo(new Vector2d(-12,-24))
                .build();
        Trajectory fourRingB = drive.trajectoryBuilder(fourRingA.end())//launch rings
                .strafeTo(new Vector2d(-3, -16))
                .build();
        Trajectory fourRingC = drive.trajectoryBuilder(fourRingB.end())//launch rings
                .strafeTo(new Vector2d(-3, -8))
                .build();
        Trajectory fourRingD = drive.trajectoryBuilder(fourRingC.end())//launch rings
                .strafeTo(new Vector2d(-3, -2))
                .build();
        Trajectory fourRingDa = drive.trajectoryBuilder(fourRingD.end())//drop wobble goal
                .lineToSplineHeading(new Pose2d(-3,-46, Math.toRadians(180)))
                .build();
        Trajectory fourRingE = drive.trajectoryBuilder(fourRingDa.end())//drop wobble goal
                .lineToSplineHeading(new Pose2d(54,-46, Math.toRadians(180)))
                .build();
        Trajectory fourRingF = drive.trajectoryBuilder(fourRingE.end())//ring stack
                .lineToSplineHeading(new Pose2d(0, -54, Math.toRadians(90)))
                .build();
        Trajectory fourRingG = drive.trajectoryBuilder(fourRingF.end())//second wobble
                .lineToConstantHeading(new Vector2d(-36, -57))
                .build();
        Trajectory fourRingH = drive.trajectoryBuilder(fourRingG.end())//ring stack
                .lineToConstantHeading(new Vector2d(0, -52))
                .build();
        Trajectory fourRingI = drive.trajectoryBuilder(fourRingH.end())// drop wobble
                .lineToSplineHeading(new Pose2d(46, -36, Math.toRadians(180)))
                .build();
        Trajectory fourRingJ = drive.trajectoryBuilder(fourRingI.end()) //park
                .lineTo(new Vector2d(12, -36))
                .build();

        //TODO add auto code here
    while (opModeIsActive()) {
        sleep(500);
        telemetry.addData("Analysis", pipeline.getAnalysis());
        telemetry.addData("Position", pipeline.position);
        telemetry.addData("RPM", rpm.getRPM());
        telemetry.update();
        if (pipeline.position == StackDeterminationPipeline.RingPosition.NONE) {
            webcam.stopStreaming();
            rpm.setRPM(2259);//launcher wheel rev up
            drive.followTrajectory(zeroRingA);
            drive.followTrajectory(zeroRingB); //launch rings
            sleep(1200);
            trigger.setPosition(.8);//set to launch
            sleep(500);
            trigger.setPosition(0.1);//launch
            sleep(550);
            trigger.setPosition(.9);
            drive.followTrajectory(zeroRingC); //launch rings
            sleep(500);
            trigger.setPosition(0.1);//launch
            sleep(550);
            trigger.setPosition(.9);
            drive.followTrajectory(zeroRingD); //launch rings
            sleep(500);
            trigger.setPosition(0.1);//launch
            sleep(550);
            trigger.setPosition(.9);
            rpm.setRPM(0);
            drive.followTrajectory(zeroRingE);//wobble goal 1
            armPower(-.8, 1.3); //arm down
            sleep(500);
            grabber(GRABBER_OPEN);
            drive.followTrajectory(zeroRingF);
            drive.followTrajectory(zeroRingG);
            drive.followTrajectory(zeroRingH);
            sleep(500);
            grabber(GRABBER_CLOSED);
            sleep(1000);
            armPower(.7,.8);//arm up
            drive.followTrajectory(zeroRingI);
            armPower(-.7, .7);//arm down
            grabber(GRABBER_OPEN);
            sleep(500);
            armPower(.7,.7);//arm up
            sleep(6000);
        } else if (pipeline.position == StackDeterminationPipeline.RingPosition.ONE) {
            webcam.stopStreaming();
            rpm.setRPM(2259);//launcher wheel rev up
            drive.followTrajectory(oneRingA);
            drive.followTrajectory(oneRingB); //launch rings
            sleep(1200);
            trigger.setPosition(.8);//set to launch
            sleep(500);
            trigger.setPosition(0.1);//launch
            sleep(550);
            trigger.setPosition(.9);
            drive.followTrajectory(oneRingC); //launch rings
            sleep(500);
            trigger.setPosition(0.1);//launch
            sleep(550);
            trigger.setPosition(.9);
            drive.followTrajectory(oneRingD); //launch rings
            sleep(500);
            trigger.setPosition(0.1);//launch
            sleep(550);
            trigger.setPosition(.9);
            drive.followTrajectory(oneRingE);//wobble goal 1
            armPower(-.5, 2.2); //arm down
            sleep(250);
            grabber(GRABBER_OPEN);
            armPower(.5,1.1);//arm up
            drive.followTrajectory(oneRingF);//ring stack
            drive.followTrajectory(oneRingG);
            armPower(-.6, 1);//arm down
            drive.followTrajectory(oneRingH);//second wobble
            grabber(GRABBER_CLOSED);
            sleep(750);
            armPower(.5,1);//arm up
            drive.followTrajectory(oneRingI);//drop second wobble
            armPower(-.5, 1);//arm down
            sleep(500);
            grabber(GRABBER_OPEN);
            sleep(500);
            armPower(.5,1);//arm up
            drive.followTrajectory(oneRingJ);//park
            sleep(6000);
        } else if (pipeline.position == StackDeterminationPipeline.RingPosition.FOUR) {
            webcam.stopStreaming();
            rpm.setRPM(2259);//launcher wheel rev up
            drive.followTrajectory(fourRingA);
            drive.followTrajectory(fourRingB); //launch rings
            sleep(1200);
            trigger.setPosition(.8);//set to launch
            sleep(500);
            trigger.setPosition(0.1);//launch
            sleep(550);
            trigger.setPosition(.9);
            drive.followTrajectory(fourRingC); //launch rings
            sleep(500);
            trigger.setPosition(0.1);//launch
            sleep(550);
            trigger.setPosition(.9);
            drive.followTrajectory(fourRingD); //launch rings
            sleep(500);
            trigger.setPosition(0.1);//launch
            sleep(550);
            trigger.setPosition(.9);
            rpm.setRPM(0);
            drive.followTrajectory(fourRingDa);
            drive.followTrajectory(fourRingE);//wobble goal 1
            armPower(-.8, 1.2); //arm down
            grabber(GRABBER_OPEN);
            sleep(600);
            armPower(.7, .75);//arm up
            drive.followTrajectory(fourRingF);//ring stack
            drive.followTrajectory(fourRingG);//second wobble
            armPower(-.8, .8);//arm DOWN
            sleep(250);
            grabber(GRABBER_CLOSED);
            sleep(600);
            armPower(.7, .8);//arm up
            drive.followTrajectory(fourRingH);//drop second wobble
            drive.followTrajectory(fourRingI);
            armPower(-.7, .7);//arm down
            sleep(250);
            grabber(GRABBER_OPEN);
            sleep(500);
            armPower(.7, .7);//arm up
            drive.followTrajectory(fourRingJ);
            sleep(6000);
        }
    }

        // Transfer the current pose to PoseStorage so we can use it in TeleOp
        PoseStorage.currentPose = drive.getPoseEstimate();
    }
    public void grabber(double position){
        grabber.setPosition(position);

    }
    public void armPower(double power, double Time){
        double time = runtime.seconds();
        while (runtime.seconds()<time + Time){
            arm.setPower(power);
        }
        arm.setPower(0);

    }
    public void launchRings(){
        sleep(5000);
        trigger.setPosition(.8);//set to launch
        sleep(500);
        trigger.setPosition(0.1);//launch
        sleep(500);
        trigger.setPosition(.9);//set to launch
        sleep(500);
        trigger.setPosition(0.1);//launch
        sleep(500);
        trigger.setPosition(.9);//set to launch
        sleep(500);
        trigger.setPosition(0.1);//launcher
        sleep(500);
        trigger.setPosition(.9);//set to launch
        sleep(500);
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
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(154,98);

        static final int REGION_WIDTH = 35;
        static final int REGION_HEIGHT = 25;

        final int FOUR_RING_THRESHOLD = 140;
        final int ONE_RING_THRESHOLD = 129;

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
    }
}

package org.firstinspires.ftc.teamcode.ultimategoal.Qualifier.Auto.Subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.ultimategoal.Qualifier.util.Hardware;
import org.firstinspires.ftc.teamcode.ultimategoal.Qualifier.util.MecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.ultimategoal.Qualifier.util.PoseStorage;
import org.firstinspires.ftc.teamcode.ultimategoal.Qualifier.util.RPMTool;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvPipeline;

@Autonomous
public class AutonomousPath extends LinearOpMode {
    Hardware2 robot           = new Hardware2();   // Use our hardware
    private ElapsedTime runtime = new ElapsedTime();

    MecanumDriveCancelable drive;
    private OpenCvCamera webcam;
    StackDeterminationPipeline pipeline;

    Pose2d startPose = new Pose2d(-63, -24, Math.toRadians(0));

    Trajectory zeroRings = drive.trajectoryBuilder(startPose)
            .lineTo(new Vector2d(0,-24)) //shoot rings
            .strafeTo(new Vector2d(6,-50)) //drop wobble goal
            .strafeTo(new Vector2d(6,-36)) //lineup to rings
            .lineToConstantHeading(new Vector2d(-48, -36)) //grab 2nd wobble goal
            .lineTo(new Vector2d(12, -36))//lineup to park
            .strafeTo(new Vector2d(12, -50))//drop 2nd wobble
            .build();
    Trajectory oneRing = drive.trajectoryBuilder(startPose)
            .lineTo(new Vector2d(0,-24))//shoot rings
            .lineTo(new Vector2d(24,-24))//drop wobble goal
            .splineToSplineHeading(new Pose2d(-8,-36, Math.toRadians(0)), Math.toRadians(26))//pickup rings
            .lineToConstantHeading(new Vector2d(-48, -36)) //grab 2nd wobble goal
            .lineToSplineHeading(new Pose2d(24,-36, Math.toRadians(90)))//drop wobble goal
            .strafeTo(new Vector2d(0,-36))//park
            .build();
    Trajectory fourRings = drive.trajectoryBuilder(startPose)
            .lineTo(new Vector2d(0,-24))//shoot rings
            .lineTo(new Vector2d(54,-48))//drop wobble goal
            .splineToSplineHeading(new Pose2d(-8,-36, Math.toRadians(0)), Math.toRadians(26))//pickup rings
            .lineToConstantHeading(new Vector2d(-48, -36)) //grab 2nd wobble goal
            .lineToConstantHeading(new Vector2d(48,-40))//drop wobble goal
            .lineToConstantHeading(new Vector2d(0,-40))//park
            .build();


    @Override
    public void runOpMode() throws InterruptedException {

        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"));
        FtcDashboard.getInstance().startCameraStream(webcam, 0);
        pipeline = new StackDeterminationPipeline();
        webcam.setPipeline(pipeline);

        drive = new MecanumDriveCancelable(hardwareMap);
        robot = new Hardware2(hardwareMap);
        robot.init(hardwareMap);

        waitForStart();

        AUTO();
    }

    public void AUTO(){
        if (opModeIsActive()) {
            double starttime = runtime.seconds();
            while (starttime + .5 >= runtime.seconds() && opModeIsActive()) {
                sleep(250);
                double ringPos = pipeline.avg1;
                telemetry.addData("Ring Pos", ringPos);
                if (ringPos <= 135) {
                    //none
                    drive.followTrajectory(zeroRings);
                    telemetry.addData("0", "rings");
                    telemetry.update();
                } else if (ringPos >= 150) {
                    //right
                    drive.followTrajectory(fourRings);
                    telemetry.addData("4", "rings");
                    telemetry.update();
                } else {
                    //one
                    drive.followTrajectory(oneRing);
                    telemetry.addData("1", "ring");
                    telemetry.update();

                }
            }
            // We update drive continuously in the background, regardless of state
            drive.update();

            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            // Continually write pose to `PoseStorage`
            PoseStorage.currentPose = poseEstimate;

            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }


    public static class StackDeterminationPipeline extends OpenCvPipeline {
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
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(181,98);

        static final int REGION_WIDTH = 35;
        static final int REGION_HEIGHT = 25;

        final int FOUR_RING_THRESHOLD = 150;
        final int ONE_RING_THRESHOLD = 135;

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

    public class Hardware2 {
        private ElapsedTime runtime = new ElapsedTime();
        private DcMotorEx launcher = null;
        private DcMotorEx arm = null;
        private DcMotorEx intake = null;
        private Servo grabber;
        private Servo trigger;


        public static final double MID_SERVO       =  0.5 ;
        public static final double highGoalRPM = 2500;
        public static final double powershotRPM = 2375;



        HardwareMap hwMap           =  null;

        /* Constructor */
        public Hardware2() {
        }

        public Hardware2(HardwareMap hardwareMap) {
        }


        /* Initialize standard Hardware interfaces */
        public void init(HardwareMap ahwMap) {
            // Save reference to Hardware map
            hwMap = ahwMap;
            RPMTool rpm = new RPMTool(launcher, 28);
            // Define and Initialize Motors
            launcher  = hwMap.get(DcMotorEx.class, "launcher");
            intake = hwMap.get(DcMotorEx.class, "intake");
            arm    = hwMap.get(DcMotorEx.class, "arm");
            launcher.setDirection(DcMotor.Direction.FORWARD);
            intake.setDirection(DcMotor.Direction.REVERSE);

            // Set all motors to zero power
            launcher.setPower(0);
            intake.setPower(0);
            arm.setPower(0);

            // Set all motors to run without encoders.
            // May want to use RUN_USING_ENCODERS if encoders are installed.
            launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            // Define and initialize ALL installed servos.
            trigger = hwMap.get(Servo.class, "trigger");
            trigger.setPosition(MID_SERVO);
            grabber  = hwMap.get(Servo.class, "grabber");
            grabber.setPosition(MID_SERVO);
        }
    }

}
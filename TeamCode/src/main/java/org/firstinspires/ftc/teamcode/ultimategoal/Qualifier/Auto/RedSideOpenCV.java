package org.firstinspires.ftc.teamcode.ultimategoal.Qualifier.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
@Disabled

@Autonomous
public class RedSideOpenCV extends LinearOpMode {
    Hardware2 robot           = new Hardware2();   // Use our hardware
    RPMTool launcher = new RPMTool(robot.launcher, 28);
    public static final double highGoalRPM = 2500;
    public static final double powershotRPM = 2375;
    public static final double GRABBER_OPEN       =  0.2 ;
    public static final double GRABBER_CLOSED       =  0.8 ;
    public static final double TRIGGER_PRESSED       =  0.5 ;
    public static final double TRIGGER_UNPRESSED       =  0.8 ;
    private ElapsedTime runtime = new ElapsedTime();

    MecanumDriveCancelable drive;
    private OpenCvCamera webcam;
    StackDeterminationPipeline pipeline;

    Pose2d startPose = new Pose2d(-63, -24, Math.toRadians(0));

    Trajectory zeroRings = drive.trajectoryBuilder(startPose)
            .lineTo(new Vector2d(0,-24))
            .addDisplacementMarker(()->{
                launcher.setRPM(highGoalRPM);
                robot.trigger.setPosition(TRIGGER_PRESSED);
                robot.trigger.setPosition(TRIGGER_UNPRESSED);
                robot.trigger.setPosition(TRIGGER_PRESSED);
                robot.trigger.setPosition(TRIGGER_UNPRESSED);
                robot.trigger.setPosition(TRIGGER_PRESSED);
                robot.trigger.setPosition(TRIGGER_UNPRESSED);
                launcher.setRPM(0);
            })
            .strafeTo(new Vector2d(0,-50))
            .lineTo(new Vector2d(6, -50))
            .addDisplacementMarker(()->{
                armPower(-.5);
                grabber(GRABBER_OPEN);
                armPower(.5);
            })
            .strafeTo(new Vector2d(6,-36))
            .lineToConstantHeading(new Vector2d(-48, -36))
            .addDisplacementMarker(()->{
                armPower(-.5);
                grabber(GRABBER_CLOSED);
                armPower(.5);
            })
            .lineTo(new Vector2d(12, -36))
            .strafeTo(new Vector2d(12, -50))
            .addDisplacementMarker(()->{
                armPower(.5);
                grabber(GRABBER_OPEN);
            })
            .build();
    Trajectory oneRing = drive.trajectoryBuilder(startPose)
            .lineTo(new Vector2d(0,-24))
            .addDisplacementMarker(()->{
                launcher.setRPM(highGoalRPM);
                robot.trigger.setPosition(TRIGGER_PRESSED);
                robot.trigger.setPosition(TRIGGER_UNPRESSED);
                robot.trigger.setPosition(TRIGGER_PRESSED);
                robot.trigger.setPosition(TRIGGER_UNPRESSED);
                robot.trigger.setPosition(TRIGGER_PRESSED);
                robot.trigger.setPosition(TRIGGER_UNPRESSED);
                launcher.setRPM(0);
            })
            .splineToConstantHeading(new Vector2d(-8,-36),Math.toRadians(0))
            .addDisplacementMarker(()->{
                intake();
                launcher.setRPM(highGoalRPM);
                robot.trigger.setPosition(TRIGGER_PRESSED);
                robot.trigger.setPosition(TRIGGER_UNPRESSED);
                launcher.setRPM(0);
            })
            //TODO add correction
            .splineTo(new Vector2d(24,-24),Math.toRadians(90))
            .addDisplacementMarker(()->{
                armPower(-.5);
                grabber(GRABBER_OPEN);
                armPower(.5);
            })
            .lineToSplineHeading(new Pose2d(-48,-36,Math.toRadians(0)))
            .addDisplacementMarker(()->{
                armPower(-.5);
                grabber(GRABBER_CLOSED);
                armPower(.5);
            })
            .lineToSplineHeading(new Pose2d(24,-36,Math.toRadians(90)))
            .addDisplacementMarker(()->{
                armPower(-.5);
                grabber(GRABBER_OPEN);
                armPower(.5);
            })
            .strafeTo(new Vector2d(6,-36))
            .build();
    Trajectory fourRings = drive.trajectoryBuilder(startPose)
            .lineTo(new Vector2d(0,-24))
            .addDisplacementMarker(()->{
                launcher.setRPM(highGoalRPM);
                robot.trigger.setPosition(TRIGGER_PRESSED);
                robot.trigger.setPosition(TRIGGER_UNPRESSED);
                robot.trigger.setPosition(TRIGGER_PRESSED);
                robot.trigger.setPosition(TRIGGER_UNPRESSED);
                robot.trigger.setPosition(TRIGGER_PRESSED);
                robot.trigger.setPosition(TRIGGER_UNPRESSED);
                launcher.setRPM(0);
            })
            .splineToConstantHeading(new Vector2d(-8,-36),Math.toRadians(0))
            .addDisplacementMarker(()->{
                intake();
                launcher.setRPM(highGoalRPM);
                robot.trigger.setPosition(TRIGGER_PRESSED);
                robot.trigger.setPosition(TRIGGER_UNPRESSED);
                robot.trigger.setPosition(TRIGGER_PRESSED);
                robot.trigger.setPosition(TRIGGER_UNPRESSED);
                robot.trigger.setPosition(TRIGGER_PRESSED);
                robot.trigger.setPosition(TRIGGER_UNPRESSED);
                robot.trigger.setPosition(TRIGGER_PRESSED);
                robot.trigger.setPosition(TRIGGER_UNPRESSED);
                launcher.setRPM(0);
            })
            //TODO Add correction
            .lineTo(new Vector2d(48,-36))
            .addDisplacementMarker(()->{
                armPower(-.5);
                grabber(GRABBER_OPEN);
                armPower(.5);
            })
            .lineToConstantHeading(new Vector2d(-48,-36))
            .addDisplacementMarker(()->{
                armPower(-.5);
                grabber(GRABBER_CLOSED);
                armPower(.5);
            })
            .lineToConstantHeading(new Vector2d(52,-36))
            .addDisplacementMarker(()->{
                armPower(-.5);
                grabber(GRABBER_OPEN);
                armPower(.5);
            })
            .lineToConstantHeading(new Vector2d(16,-36))
            .build();
//Zero RINGS
    Trajectory zeroRingsA = drive.trajectoryBuilder(startPose)
            .lineTo(new Vector2d(0,-24))
            .addDisplacementMarker(()->{
                launcher.setRPM(highGoalRPM);
                robot.trigger.setPosition(TRIGGER_PRESSED);
                robot.trigger.setPosition(TRIGGER_UNPRESSED);
                robot.trigger.setPosition(TRIGGER_PRESSED);
                robot.trigger.setPosition(TRIGGER_UNPRESSED);
                robot.trigger.setPosition(TRIGGER_PRESSED);
                robot.trigger.setPosition(TRIGGER_UNPRESSED);
                launcher.setRPM(0);
            })
            .build();
    Trajectory zeroRingsB = drive.trajectoryBuilder(zeroRingsA.end())
            .strafeTo(new Vector2d(0,-50))
            .lineTo(new Vector2d(6, -50))
            .addDisplacementMarker(()->{
                armPower(-.5);
                grabber(GRABBER_OPEN);
                armPower(.5);
            })
            .build();
    Trajectory zeroRingsC = drive.trajectoryBuilder(zeroRingsB.end())
            .strafeTo(new Vector2d(6,-36))
            .lineToConstantHeading(new Vector2d(-48, -36))
            .addDisplacementMarker(()->{
                armPower(-.5);
                grabber(GRABBER_CLOSED);
                armPower(.5);
            })
            .build();
    Trajectory zeroRingsD = drive.trajectoryBuilder(zeroRingsC.end())
            .lineTo(new Vector2d(12, -36))
            .strafeTo(new Vector2d(12, -50))
            .addDisplacementMarker(()->{
                armPower(.5);
                grabber(GRABBER_OPEN);
            })
            .build();
//One RING
    Trajectory oneRingA = drive.trajectoryBuilder(startPose)
            .lineTo(new Vector2d(0,-24))
            .addDisplacementMarker(()->{
                launcher.setRPM(highGoalRPM);
                robot.trigger.setPosition(TRIGGER_PRESSED);
                robot.trigger.setPosition(TRIGGER_UNPRESSED);
                robot.trigger.setPosition(TRIGGER_PRESSED);
                robot.trigger.setPosition(TRIGGER_UNPRESSED);
                robot.trigger.setPosition(TRIGGER_PRESSED);
                robot.trigger.setPosition(TRIGGER_UNPRESSED);
                launcher.setRPM(0);
            })
            .build();
    Trajectory oneRingB = drive.trajectoryBuilder(oneRingA.end())
            .splineToConstantHeading(new Vector2d(-8,-36),Math.toRadians(0))
            .addDisplacementMarker(()->{
                intake();
                launcher.setRPM(highGoalRPM);
                robot.trigger.setPosition(TRIGGER_PRESSED);
                robot.trigger.setPosition(TRIGGER_UNPRESSED);
                launcher.setRPM(0);
            })
            .build();
    Trajectory oneRingC = drive.trajectoryBuilder(oneRingB.end())
            //TODO add correction
            .splineTo(new Vector2d(24,-24),Math.toRadians(90))
            .addDisplacementMarker(()->{
                armPower(-.5);
                grabber(GRABBER_OPEN);
                armPower(.5);
            })
            .build();
    Trajectory oneRingD = drive.trajectoryBuilder(oneRingC.end())
            .lineToSplineHeading(new Pose2d(-48,-36,Math.toRadians(0)))
            .addDisplacementMarker(()->{
                armPower(-.5);
                grabber(GRABBER_CLOSED);
                armPower(.5);
            })
            .build();
    Trajectory oneRingE = drive.trajectoryBuilder(oneRingD.end())
            .lineToSplineHeading(new Pose2d(24,-36,Math.toRadians(90)))
            .addDisplacementMarker(()->{
                armPower(-.5);
                grabber(GRABBER_OPEN);
                armPower(.5);
            })
            .build();
    Trajectory oneRingF = drive.trajectoryBuilder(oneRingE.end())
            .strafeTo(new Vector2d(6,-36))
            .build();
//Four RINGS
    Trajectory fourRingsA = drive.trajectoryBuilder(startPose)
            .lineTo(new Vector2d(0,-24))
            .addDisplacementMarker(()->{
                launcher.setRPM(highGoalRPM);
                robot.trigger.setPosition(TRIGGER_PRESSED);
                robot.trigger.setPosition(TRIGGER_UNPRESSED);
                robot.trigger.setPosition(TRIGGER_PRESSED);
                robot.trigger.setPosition(TRIGGER_UNPRESSED);
                robot.trigger.setPosition(TRIGGER_PRESSED);
                robot.trigger.setPosition(TRIGGER_UNPRESSED);
                launcher.setRPM(0);
            })
            .build();
    Trajectory fourRingsB = drive.trajectoryBuilder(fourRingsA.end())
            .splineToConstantHeading(new Vector2d(-8,-36),Math.toRadians(0))
            .addDisplacementMarker(()->{
                intake();
                launcher.setRPM(highGoalRPM);
                robot.trigger.setPosition(TRIGGER_PRESSED);
                robot.trigger.setPosition(TRIGGER_UNPRESSED);
                robot.trigger.setPosition(TRIGGER_PRESSED);
                robot.trigger.setPosition(TRIGGER_UNPRESSED);
                robot.trigger.setPosition(TRIGGER_PRESSED);
                robot.trigger.setPosition(TRIGGER_UNPRESSED);
                robot.trigger.setPosition(TRIGGER_PRESSED);
                robot.trigger.setPosition(TRIGGER_UNPRESSED);
                launcher.setRPM(0);
            })
            .build();
    Trajectory fourRingsC = drive.trajectoryBuilder(fourRingsB.end())
            //TODO Add correction
            .lineTo(new Vector2d(48,-36))
            .addDisplacementMarker(()->{
                armPower(-.5);
                grabber(GRABBER_OPEN);
                armPower(.5);
            })
            .build();
    Trajectory fourRingsD = drive.trajectoryBuilder(fourRingsC.end())
            .lineToConstantHeading(new Vector2d(-48,-36))
            .addDisplacementMarker(()->{
                armPower(-.5);
                grabber(GRABBER_CLOSED);
                armPower(.5);
            })
            .build();
    Trajectory fourRingsE = drive.trajectoryBuilder(fourRingsD.end())
            .lineToConstantHeading(new Vector2d(52,-36))
            .addDisplacementMarker(()->{
                armPower(-.5);
                grabber(GRABBER_OPEN);
                armPower(.5);
            })
            .build();
    Trajectory fourRingsF = drive.trajectoryBuilder(fourRingsE.end())
            .lineToConstantHeading(new Vector2d(16,-36))
            .build();

    @Override
    public void runOpMode() throws InterruptedException {

        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));
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

    public void AUTO2(){
        if (opModeIsActive()) {
            double starttime = runtime.seconds();
            while (starttime + .5 >= runtime.seconds() && opModeIsActive()) {
                sleep(250);
                double ringPos = pipeline.avg1;
                telemetry.addData("Ring Pos", ringPos);
                if (ringPos <= 135) {
                    //none
                    drive.followTrajectory(zeroRingsA);
                    drive.followTrajectory(zeroRingsB);
                    drive.followTrajectory(zeroRingsC);
                    drive.followTrajectory(zeroRingsD);
                    telemetry.addData("0", "rings");
                    telemetry.update();
                } else if (ringPos >= 150) {
                    //right
                    drive.followTrajectory(fourRingsA);
                    drive.followTrajectory(fourRingsB);
                    drive.followTrajectory(fourRingsC);
                    drive.followTrajectory(fourRingsD);
                    drive.followTrajectory(fourRingsE);
                    drive.followTrajectory(fourRingsF);
                    telemetry.addData("4", "rings");
                    telemetry.update();
                } else {
                    //one
                    drive.followTrajectory(oneRingA);
                    drive.followTrajectory(oneRingB);
                    drive.followTrajectory(oneRingC);
                    drive.followTrajectory(oneRingD);
                    drive.followTrajectory(oneRingE);
                    drive.followTrajectory(oneRingF);
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

    public void armPower(double power){
            double time = runtime.seconds();
            while (runtime.seconds()<time + 1){
                robot.arm.setPower(power);
            }
            robot.arm.setPower(0);

    }

    public void grabber(double position){
        robot.grabber.setPosition(position);

    }

    public void intake(){
        double time = runtime.seconds();
        drive.setMotorPowers(.1,.1,.1,.1);
        while (runtime.seconds()<time + 1){
            robot.intake.setPower(.5);
        }
        drive.setMotorPowers(0,0,0,0);
        robot.intake.setPower(0);
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
            launcher  = hwMap.get(DcMotorEx.class, "left_drive");
            intake = hwMap.get(DcMotorEx.class, "right_drive");
            arm    = hwMap.get(DcMotorEx.class, "left_arm");
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
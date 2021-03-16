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
public class Arm extends LinearOpMode {
    Hardware2 robot           = new Hardware2();   // Use our hardware
    public static final double GRABBER_OPEN       =  0.4 ;
    public static final double GRABBER_CLOSED       =  0.8 ;
    int ARM_UP = -40;
    int ARM_DOWN = 40;
    private ElapsedTime runtime = new ElapsedTime();

    MecanumDriveCancelable drive;

    Pose2d startPose = new Pose2d(-63, -24, Math.toRadians(0));

    Trajectory zeroRings = drive.trajectoryBuilder(startPose)
            .lineTo(new Vector2d(0,-24))
            .strafeTo(new Vector2d(6,-50))
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
    Trajectory zeroRings1 = drive.trajectoryBuilder(startPose)
            .lineTo(new Vector2d(0,-24))
            .strafeTo(new Vector2d(6,-50))
            .addDisplacementMarker(()->{
                armPosition(100);
                grabber(GRABBER_OPEN);
                armPosition(ARM_UP);
            })
            .strafeTo(new Vector2d(6,-36))
            .lineToConstantHeading(new Vector2d(-48, -36))
            .addDisplacementMarker(()->{
                armPower(ARM_DOWN);
                grabber(GRABBER_CLOSED);
                armPower(ARM_UP);
            })
            .lineTo(new Vector2d(12, -36))
            .strafeTo(new Vector2d(12, -50))
            .addDisplacementMarker(()->{
                armPower(ARM_DOWN);
                grabber(GRABBER_OPEN);
            })
            .build();

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new MecanumDriveCancelable(hardwareMap);
        robot = new Hardware2(hardwareMap);
        robot.init(hardwareMap);

        waitForStart();

        AUTO();
    }

    public void AUTO(){
        if (opModeIsActive()) {
            drive.followTrajectory(zeroRings1);
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
    public void armPosition(int angle){
        int ticks = robot.arm.getCurrentPosition();
        int ticksPerRevInput = 28;
        int gearRatio = 256;
        int ticksPerRevOutput = ticksPerRevInput*gearRatio;
        int ticksPerAngle = (int) (3.141592654/ticksPerRevOutput);
        int target = angle * ticksPerAngle;

        robot.arm.setTargetPosition(ticks + target);


    }


    public void armPower(double power){
            double time = runtime.seconds();
            while (runtime.seconds()<time + .5){
                robot.arm.setPower(power);
            }
            robot.arm.setPower(0);

    }

    public void grabber(double position){
        robot.grabber.setPosition(position);

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
            intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            // Define and initialize ALL installed servos.
            trigger = hwMap.get(Servo.class, "trigger");
            trigger.setPosition(MID_SERVO);
            grabber  = hwMap.get(Servo.class, "grabber");
            grabber.setPosition(MID_SERVO);
        }
    }

}
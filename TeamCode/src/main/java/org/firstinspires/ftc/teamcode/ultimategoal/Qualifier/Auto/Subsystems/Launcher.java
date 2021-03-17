package org.firstinspires.ftc.teamcode.ultimategoal.Qualifier.Auto.Subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.ultimategoal.Qualifier.util.Hardware;
import org.firstinspires.ftc.teamcode.ultimategoal.Qualifier.util.MecanumDrive;
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
public class Launcher extends LinearOpMode {
    Hardware2 robot           = new Hardware2();   // Use our hardware
    RPMTool launcher = new RPMTool(robot.launcher, 28);
    public static final double highGoalRPM = 2500;
    public static final double powershotRPM = 2375;
    public static final double GRABBER_OPEN       =  0.2 ;
    public static final double GRABBER_CLOSED       =  0.8 ;
    public static final double TRIGGER_PRESSED       =  0.1 ;
    public static final double TRIGGER_UNPRESSED       =  0.9 ;
    private ElapsedTime runtime = new ElapsedTime();
    private Servo trigger;

    double setMotorTime = 1; // What time we set the motor power
    double setMotorWait = 1.5; // How long we wait until we launch the rings

    MecanumDrive drive;


    Pose2d startPose = new Pose2d(-63, -24, Math.toRadians(0));

    Trajectory zeroRings = drive.trajectoryBuilder(startPose)
            .lineTo(new Vector2d(0,-24))
            .addDisplacementMarker(()->{
                launcher.setRPM(highGoalRPM);
            })
            .addTemporalMarker(setMotorTime+ setMotorWait, () -> {
                trigger.setPosition(TRIGGER_PRESSED);
                trigger.setPosition(TRIGGER_UNPRESSED);
                trigger.setPosition(TRIGGER_PRESSED);
                trigger.setPosition(TRIGGER_UNPRESSED);
                trigger.setPosition(TRIGGER_PRESSED);
                trigger.setPosition(TRIGGER_UNPRESSED);
                launcher.setRPM(0);
            })
            .strafeTo(new Vector2d(0,-50))
            .lineTo(new Vector2d(6, -50))
            .build();

    Trajectory zeroRings1 = drive.trajectoryBuilder(startPose)
            .lineTo(new Vector2d(0,-24))
            .addDisplacementMarker(()-> {
                        launcher.setRPM(highGoalRPM);
                    })
            .addTemporalMarker(setMotorTime+ setMotorWait, () -> {
                trigger.setPosition(TRIGGER_PRESSED);
                trigger.setPosition(TRIGGER_UNPRESSED);
                trigger.setPosition(TRIGGER_PRESSED);
                trigger.setPosition(TRIGGER_UNPRESSED);
                trigger.setPosition(TRIGGER_PRESSED);
                trigger.setPosition(TRIGGER_UNPRESSED);
                launcher.setRPM(0);
            })
            .strafeTo(new Vector2d(0,-50))
            .lineTo(new Vector2d(6, -50))
            .build();

    @Override
    public void runOpMode() throws InterruptedException {



        drive = new MecanumDrive(hardwareMap);
        robot = new Hardware2(hardwareMap);
        robot.init(hardwareMap);
//
//        DcMotorEx launcher = hardwareMap.get(DcMotorEx.class, "launcher");
//        launcher.setDirection(DcMotorSimple.Direction.FORWARD);
//        RPMTool rpm = new RPMTool(launcher, 28);

        waitForStart();

        AUTO();
    }

    public void AUTO(){
        if (opModeIsActive()) {
            drive.followTrajectory(zeroRings);
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
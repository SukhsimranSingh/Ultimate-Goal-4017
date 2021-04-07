package org.firstinspires.ftc.teamcode.ultimategoal.Qualifier.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.vision.UGContourRingPipeline;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.advanced.PoseStorage;
import org.firstinspires.ftc.teamcode.ultimategoal.Qualifier.util.RPMTool;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

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
public class AutoChamps extends LinearOpMode {
    private Servo trigger;
    private Servo grabber;
    private DcMotorEx arm;
    private DcMotor intake;
    private ElapsedTime runtime = new ElapsedTime();
    public static final double GRABBER_OPEN       =  0.1;
    public static final double GRABBER_CLOSED       =  0.9;
    public static double TRIGGER_PRESSED    =  0.55 ;
    public static final double TRIGGER_UNPRESSED  =  0.75 ;

    double powerShotRPM = 3450;
    double HIGHGOAL = 4150;

    private static final int CAMERA_WIDTH = 320; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 240; // height of wanted camera resolution

    private static final int HORIZON = 100; // horizon value to tune

    private static final boolean DEBUG = false; // if debug is wanted, change to true

    private static final boolean USING_WEBCAM = true; // change to true if using webcam
    private static final String WEBCAM_NAME = "webcam"; // insert webcam name from configuration if using webcam

    private UGContourRingPipeline pipeline;
    private OpenCvCamera camera;


    @Override
    public void runOpMode() throws InterruptedException {
        // Declare your drive class
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        DcMotorEx launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        arm    = hardwareMap.get(DcMotorEx.class, "arm");
        intake = hardwareMap.get(DcMotorEx.class, "intake");

        launcher.setDirection(DcMotorSimple.Direction.REVERSE);

        RPMTool rpm = new RPMTool(launcher, 28);
        grabber  = hardwareMap.servo.get("grabber");
        trigger = hardwareMap.servo.get("trigger");

        int cameraMonitorViewId = this
                .hardwareMap
                .appContext
                .getResources().getIdentifier(
                        "cameraMonitorViewId",
                        "id",
                        hardwareMap.appContext.getPackageName()
                );
        if (USING_WEBCAM) {
            camera = OpenCvCameraFactory
                    .getInstance()
                    .createWebcam(hardwareMap.get(WebcamName.class, WEBCAM_NAME), cameraMonitorViewId);
        } else {
            camera = OpenCvCameraFactory
                    .getInstance()
                    .createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        }
        FtcDashboard.getInstance().startCameraStream(camera, 0);

        camera.setPipeline(pipeline = new UGContourRingPipeline(telemetry, DEBUG));

        UGContourRingPipeline.Config.setCAMERA_WIDTH(CAMERA_WIDTH);

        UGContourRingPipeline.Config.setHORIZON(HORIZON);

        camera.openCameraDeviceAsync(() -> camera.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT));


        // Set the pose estimate to where you know the bot will start in autonomous
        // Refer to https://www.learnroadrunner.com/trajectories.html#coordinate-system for a map
        // of the field
        // This example sets the bot at x: 10, y: 15, and facing 90 degrees (turned counter-clockwise)
        Pose2d startPose = new Pose2d(-63, -24, Math.toRadians(0));

        drive.setPoseEstimate(startPose);
        grabber.setPosition(GRABBER_CLOSED);
        trigger.setPosition(TRIGGER_UNPRESSED);

        waitForStart();

        if (isStopRequested()) return;
        // Example spline path from SplineTest.java
        // Make sure the start pose matches with the localizer's start pose
        Trajectory zeroRingA = drive.trajectoryBuilder(startPose)//to launch line
                .lineTo(new Vector2d(-12,-24))
                .build();
        Trajectory zeroRingB = drive.trajectoryBuilder(zeroRingA.end())//launch rings
                .strafeTo(new Vector2d(-12, -16))
                .build();
        Trajectory zeroRingC = drive.trajectoryBuilder(zeroRingB.end())//powershot 1
                .strafeTo(new Vector2d(-12, -8))
                .build();
        Trajectory zeroRingD = drive.trajectoryBuilder(zeroRingC.end())//powershot 2
                .strafeTo(new Vector2d(-12, -2))
                .build();
        Trajectory zeroRingE = drive.trajectoryBuilder(zeroRingD.end())
                .lineToSplineHeading(new Pose2d(18,-50, Math.toRadians(180))) //drop wobble goal
                .build();
        Trajectory zeroRingF = drive.trajectoryBuilder(zeroRingE.end())//going to second
                .lineTo(new Vector2d(18, -36))
                .build();
        Trajectory zeroRingG = drive.trajectoryBuilder(zeroRingF.end())//lining up
                .strafeTo(new Vector2d(-47,-36))
                .build();
        Trajectory zeroRingH = drive.trajectoryBuilder(zeroRingG.end())// second wobble
                .lineToConstantHeading(new Vector2d(17, -35))
                .build();


        Trajectory oneRingA = drive.trajectoryBuilder(startPose)//to launch line
                .lineTo(new Vector2d(-12,-24))
                .build();
        Trajectory oneRingB = drive.trajectoryBuilder(oneRingA.end())//powershot 1
                .strafeTo(new Vector2d(-12, -16))
                .build();
        Trajectory oneRingC = drive.trajectoryBuilder(oneRingB.end())//powershot 2
                .strafeTo(new Vector2d(-12, -8))
                .build();
        Trajectory oneRingD = drive.trajectoryBuilder(oneRingC.end())//powershot 3
                .strafeTo(new Vector2d(-12, -2))
                .build();
        Trajectory oneRingE = drive.trajectoryBuilder(oneRingD.end())
                .lineToSplineHeading(new Pose2d(18,-26, Math.toRadians(-90))) //drop wobble goal
                .build();
        Trajectory oneRingF = drive.trajectoryBuilder(oneRingE.end())//ring stack
                .lineToSplineHeading(new Pose2d(-16, -36, Math.toRadians(0)))
                .build();
        Trajectory oneRingG = drive.trajectoryBuilder(oneRingF.end())//
                .lineToSplineHeading(new Pose2d(-44, -33, Math.toRadians(180)))
                .build();
        Trajectory oneRingH = drive.trajectoryBuilder(oneRingG.end())// second wobble
                .lineToSplineHeading(new Pose2d(12, -38, Math.toRadians(-90)))
                .build();


        Trajectory fourRingA = drive.trajectoryBuilder(startPose)//to launch line
                .lineTo(new Vector2d(-12,-24))
                .build();
        Trajectory fourRingB = drive.trajectoryBuilder(fourRingA.end())//launch rings
                .strafeTo(new Vector2d(-12, -16))
                .build();
        Trajectory fourRingC = drive.trajectoryBuilder(fourRingB.end())//launch rings
                .strafeTo(new Vector2d(-12, -8))
                .build();
        Trajectory fourRingD = drive.trajectoryBuilder(fourRingC.end())//launch rings
                .strafeTo(new Vector2d(-12, -2))
                .build();
        Trajectory fourRingDa = drive.trajectoryBuilder(fourRingD.end())//drop wobble goal
                .lineToSplineHeading(new Pose2d(-3,-46, Math.toRadians(180)))
                .build();
        Trajectory fourRingE = drive.trajectoryBuilder(fourRingDa.end())//drop wobble goal
                .lineToSplineHeading(new Pose2d(54,-46, Math.toRadians(180)))
                .build();
        Trajectory fourRingF = drive.trajectoryBuilder(fourRingE.end())//ring stack
                .lineToSplineHeading(new Pose2d(0, -54, Math.toRadians(94)))
                .build();
        Trajectory fourRingG = drive.trajectoryBuilder(fourRingF.end())//second wobble
                .lineToConstantHeading(new Vector2d(-35, -57))
                .build();
        Trajectory fourRingH = drive.trajectoryBuilder(fourRingG.end())//ring stack
                .lineToConstantHeading(new Vector2d(0, -52))
                .build();
        Trajectory fourRingI = drive.trajectoryBuilder(fourRingH.end())// drop wobble
                .lineToSplineHeading(new Pose2d(44, -38, Math.toRadians(180)))
                .build();
        Trajectory fourRingJ = drive.trajectoryBuilder(fourRingI.end()) //park
                .lineTo(new Vector2d(12, -38))
                .build();
        //common path
        Trajectory launchLine = drive.trajectoryBuilder(startPose)//to launch line
                .lineTo(new Vector2d(-12,-24))
                .build();
        Trajectory powershotOne = drive.trajectoryBuilder(launchLine.end())//powershot 1
                .strafeTo(new Vector2d(-12, -16))
                .build();
        Trajectory powershotTwo = drive.trajectoryBuilder(powershotOne.end())//powershot 2
                .strafeTo(new Vector2d(-12, -8))
                .build();
        Trajectory powershotThree = drive.trajectoryBuilder(powershotTwo.end())//powershot 3
                .strafeTo(new Vector2d(-12, -2))
                .build();
        //zero rings path
        Trajectory AZeroRings = drive.trajectoryBuilder(powershotThree.end())
                .lineToSplineHeading(new Pose2d(18,-50, Math.toRadians(180))) //drop wobble goal
                .build();
        Trajectory BZeroRings = drive.trajectoryBuilder(AZeroRings.end())//going to second
                .lineTo(new Vector2d(18, -36))
                .build();
        Trajectory CZeroRings = drive.trajectoryBuilder(BZeroRings.end())//lining up
                .strafeTo(new Vector2d(-48,-36))
                .build();
        Trajectory DZeroRings = drive.trajectoryBuilder(CZeroRings.end())// second wobble
                .lineToConstantHeading(new Vector2d(18, -35))
                .build();
        //one ring path
        Trajectory AOneRing = drive.trajectoryBuilder(powershotThree.end())
                .lineToSplineHeading(new Pose2d(18,-26, Math.toRadians(-90))) //drop wobble goal
                .build();
        Trajectory BOneRing = drive.trajectoryBuilder(AOneRing.end())//ring stack
                .lineToSplineHeading(new Pose2d(-16, -36, Math.toRadians(0)))
                .build();
        Trajectory COneRing = drive.trajectoryBuilder(BOneRing.end())//
                .lineToSplineHeading(new Pose2d(-50, -36, Math.toRadians(180)))
                .build();
        Trajectory DOneRing = drive.trajectoryBuilder(COneRing.end())// second wobble
                .lineToSplineHeading(new Pose2d(12, -36, Math.toRadians(-90)))
                .build();
        //four ring path
        Trajectory AFourRings = drive.trajectoryBuilder(powershotThree.end())//lineup to drop wobble goal
                .lineToSplineHeading(new Pose2d(-3,-36, Math.toRadians(180)))
                .build();
        Trajectory BFourRings = drive.trajectoryBuilder(AFourRings.end())//drop wobble goal
                .lineToConstantHeading(new Vector2d(54, -36))
                .build();
        Trajectory CFourRings = drive.trajectoryBuilder(BFourRings.end())//ring stack
                .lineToSplineHeading(new Pose2d(-24, -36, Math.toRadians(0)))
                .build();
        Trajectory DFourRings = drive.trajectoryBuilder(CFourRings.end())//second wobble
                .lineToSplineHeading(new Pose2d(-48, -36, Math.toRadians(180)))
                .build();
        Trajectory EFourRings = drive.trajectoryBuilder(DFourRings.end())//drop second wobble
                .lineToConstantHeading(new Vector2d(50, -36))
                .build();
        Trajectory FFourRings = drive.trajectoryBuilder(EFourRings.end())//park
                .lineTo(new Vector2d(12, -36))
                .build();

    while (opModeIsActive() && !isStopRequested()) {
        sleep(500);
        String height = "[HEIGHT]" + " " + pipeline.getHeight();
        telemetry.addData("[Ring Stack] >>", height);
        telemetry.addData("RPM", rpm.getRPM());
        telemetry.update();
        //REAL AUTO
        if (pipeline.getHeight() == UGContourRingPipeline.Height.ZERO) {// TODO CHAGNGE TO ZERO
            rpm.setRPM(powerShotRPM);//launcher wheel rev up
            drive.followTrajectory(zeroRingA);
            drive.followTrajectory(zeroRingB); //powershot 1
            sleep(750);
            trigger.setPosition(TRIGGER_UNPRESSED);//set to launch
            sleep(500);
            trigger.setPosition(TRIGGER_PRESSED);//launch
            sleep(550);
            trigger.setPosition(TRIGGER_UNPRESSED);
            drive.followTrajectory(zeroRingC); //powershot 2
            sleep(550);
            trigger.setPosition(TRIGGER_PRESSED);//launch
            sleep(550);
            trigger.setPosition(TRIGGER_UNPRESSED);
            drive.followTrajectory(zeroRingD); //powershot 3
            sleep(500);
            trigger.setPosition(TRIGGER_PRESSED);//launch
            sleep(550);
            trigger.setPosition(TRIGGER_UNPRESSED);
            rpm.setRPM(0);
            drive.followTrajectory(zeroRingE);//wobble goal 1
            armPower(-.8, 1.45); //arm down
            sleep(500);
            grabber(GRABBER_OPEN);
            drive.followTrajectory(zeroRingF);
            drive.followTrajectory(zeroRingG);//second wobble
            sleep(500);
            grabber(GRABBER_CLOSED);
            sleep(1000);
            armPower(.7,.7);//arm up
            drive.followTrajectory(zeroRingH);// drop 2nd wobble and park
            armPower(-.7, .7);//arm down
            grabber(GRABBER_OPEN);
            sleep(500);
            armPower(.7,.7);//arm up
            sleep(6000);

        } else if (pipeline.getHeight() == UGContourRingPipeline.Height.ONE) {//TODO CHANGE TO ONE
            rpm.setRPM(powerShotRPM);//launcher wheel rev up
            drive.followTrajectory(oneRingA);
            drive.followTrajectory(oneRingB); //powershot 1
           sleep(750);
            trigger.setPosition(TRIGGER_UNPRESSED);//set to launch
            sleep(500);
            trigger.setPosition(TRIGGER_PRESSED);//launch
            sleep(550);
            trigger.setPosition(TRIGGER_UNPRESSED);
            drive.followTrajectory(oneRingC); //powershot 2
            sleep(550);
            trigger.setPosition(TRIGGER_PRESSED);//launch
            sleep(550);
            trigger.setPosition(TRIGGER_UNPRESSED);
            drive.followTrajectory(oneRingD); //powershot 3
            sleep(500);
            trigger.setPosition(TRIGGER_PRESSED);//launch
            sleep(550);
            trigger.setPosition(TRIGGER_UNPRESSED);
            rpm.setRPM(0);
            drive.followTrajectory(oneRingE);//drop wobble 1
            armPower(-.8, 1.4); //arm down
            grabber(GRABBER_OPEN);
            sleep(500);
            armPower(.7,.7);//arm up
            rpm.setRPM(HIGHGOAL + 100);
            intake.setPower(-1);
            drive.followTrajectory(oneRingF);//ringstack
            intake(1, .5);
            sleep(750);
            trigger.setPosition(TRIGGER_PRESSED);//launch
            sleep(550);
            trigger.setPosition(TRIGGER_UNPRESSED);
            rpm.setRPM(0);
            drive.followTrajectory(oneRingG);//second wobble
            armPower(-.7, .75);//arm down
            sleep(500);
            grabber(GRABBER_CLOSED);
            sleep(750);
            armPower(.7,.7);//arm up
            drive.followTrajectory(oneRingH);//drop wobble 2 and park
            armPower(-.7,.7);//arm down
            grabber(GRABBER_OPEN);
            sleep(500);
            armPower(.7,.7);//arm up
            sleep(6000);
        } else if (pipeline.getHeight() == UGContourRingPipeline.Height.FOUR) {
            rpm.setRPM(powerShotRPM);//launcher wheel rev up
            drive.followTrajectory(fourRingA);
            drive.followTrajectory(fourRingB); //launch rings
            sleep(550);
            trigger.setPosition(TRIGGER_UNPRESSED);//set to launch
            trigger.setPosition(TRIGGER_PRESSED);//launch
            sleep(550);
            trigger.setPosition(TRIGGER_UNPRESSED);
            drive.followTrajectory(fourRingC); //powershot 2
            sleep(550);
            trigger.setPosition(TRIGGER_PRESSED);//launch
            sleep(550);
            trigger.setPosition(TRIGGER_UNPRESSED);
            drive.followTrajectory(fourRingD); //powershot 3
            sleep(500);
            trigger.setPosition(TRIGGER_PRESSED);//launch
            sleep(550);
            trigger.setPosition(TRIGGER_UNPRESSED);
            rpm.setRPM(0);
            drive.followTrajectory(fourRingDa);
            drive.followTrajectory(fourRingE);//wobble goal 1
            armPower(-.8, 1.25); //arm down
            grabber(GRABBER_OPEN);
            sleep(250);
            armPower(.7, .7);//arm up
            drive.followTrajectory(fourRingF);//ring stack
            drive.followTrajectory(fourRingG);//second wobble
            armPower(-.85, .85);//arm DOWN
            grabber(GRABBER_CLOSED);
            sleep(500);
            armPower(.7, .8);//arm up
            drive.followTrajectory(fourRingH);//drop second wobble
            drive.followTrajectory(fourRingI);
            armPower(-.7, .7);//arm down
            grabber(GRABBER_OPEN);
            sleep(250);
            armPower(.7, .7);//arm up
            drive.followTrajectory(fourRingJ);
            sleep(6000);
        }

        //TODO synchronization for auto
//        if (pipeline.getHeight() == UGContourRingPipeline.Height.ZERO) {
//            rpm.setRPM(powerShotRPM);//launcher wheel rev up
//            drive.followTrajectory(launchLine); //launchline
//            drive.followTrajectory(powershotOne);//powershot1
//            launchRing();
//            drive.followTrajectory(powershotTwo);//powershot2
//            launchRing();
//            drive.followTrajectory(powershotThree);//powershot3
//            launchRing();
//            rpm.setRPM(0);
//            drive.followTrajectory(AZeroRings);//drop wobble goal 1
//            WobbleGoal1();
//            drive.followTrajectory(BZeroRings);//lineup wobble goal 2
//            drive.followTrajectory(CZeroRings);//pickup wobble 2
//            grabber(GRABBER_CLOSED);
//            sleep(500);
//            armUp();
//            drive.followTrajectory(DZeroRings);//drop wobble 2
//            armDown();
//            grabber(GRABBER_OPEN);
//            sleep(6000);
//
//        } else if (pipeline.getHeight() == UGContourRingPipeline.Height.ONE) {
//            rpm.setRPM(powerShotRPM);//launcher wheel rev up
//            drive.followTrajectory(launchLine); //launchline
//            drive.followTrajectory(powershotOne);//powershot1
//            launchRing();
//            drive.followTrajectory(powershotTwo);//powershot2
//            launchRing();
//            drive.followTrajectory(powershotThree);//powershot3
//            launchRing();
//            rpm.setRPM(0);
//            drive.followTrajectory(AOneRing);//drop wobble goal 1
//            WobbleGoal1();
//            armUp();
//            intake(.8);
//            drive.followTrajectory(BOneRing);//ring stack
//            intake(0);
//            rpm.setRPM(HIGHGOAL);
//            sleep(750);
//            launchRing();
//            rpm.setRPM(0);
//            drive.followTrajectory(COneRing);//pickup wobble 2
//            armDown();
//            grabber(GRABBER_CLOSED);
//            sleep(500);
//            armUp();
//            drive.followTrajectory(DOneRing);//drop wobble 2
//            armDown();
//            grabber(GRABBER_OPEN);
//            sleep(6000);
//
//        } else if (pipeline.getHeight() == UGContourRingPipeline.Height.FOUR) {
//            rpm.setRPM(powerShotRPM);//launcher wheel rev up
//            drive.followTrajectory(launchLine); //launchline
//            drive.followTrajectory(powershotOne);//powershot1
//            launchRing();
//            drive.followTrajectory(powershotTwo);//powershot2
//            launchRing();
//            drive.followTrajectory(powershotThree);//powershot3
//            launchRing();
//            rpm.setRPM(0);
//            drive.followTrajectory(AFourRings);//lineup to wobblw goal
//            drive.followTrajectory(BFourRings);//drop wobble goal 1
//            WobbleGoal1();
//            armUp();
//            intake(.8);
//            drive.followTrajectory(CFourRings);//ring stack
//            intake(0);
//            rpm.setRPM(HIGHGOAL);
//            sleep(750);
//            launchRing();
//            launchRing();
//            launchRing();
//            launchRing();
//            rpm.setRPM(0);
//            drive.followTrajectory(DFourRings);//pickup wobble 2
//            armDown();
//            grabber(GRABBER_CLOSED);
//            sleep(500);
//            armUp();
//            drive.followTrajectory(EFourRings);//drop wobble 2
//            armDown();
//            grabber(GRABBER_OPEN);
//            armUp();
//            drive.followTrajectory(FFourRings);//park
//            sleep(6000);
//        }
    }

        // Transfer the current pose to PoseStorage so we can use it in TeleOp
        PoseStorage.currentPose = drive.getPoseEstimate();
    }
    public void grabber(double position){
        grabber.setPosition(position);

    }
    public void armPower(double power, double Time){
        double time = runtime.seconds();
        while (runtime.seconds()<time + Time && opModeIsActive()){
            arm.setPower(power);
        }
        arm.setPower(0);

    }
    public void WobbleGoal1(){
        armPower(-.8,1.3);
        grabber(GRABBER_OPEN);
        sleep(500);
    }
    public void armDown(){
        armPower(-.7,.7);
    }
    public void armUp(){
        armPower(.7,.7);
    }
    public void launchRing(){
        trigger.setPosition(TRIGGER_PRESSED);//launch
        sleep(500);
        trigger.setPosition(TRIGGER_UNPRESSED);//set to launch
        sleep(250);

    }
    public void intake(double power, double time){
        double Time = runtime.seconds();
        while (runtime.seconds()<time + Time && opModeIsActive()){
//            power = Math.abs(power);
            intake.setPower(-power);
        }
        intake.setPower(0);
    }

}

package org.firstinspires.ftc.teamcode.ultimategoal.Qualifier.Auto.Paths;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.advanced.PoseStorage;
import org.firstinspires.ftc.teamcode.ultimategoal.Qualifier.util.RPMTool;

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
public class AutoFourRings extends LinearOpMode {
    private Servo trigger;
    private Servo grabber;
    private DcMotorEx arm;
    private ElapsedTime runtime = new ElapsedTime();
    public static final double GRABBER_OPEN       =  0.1;
    public static final double GRABBER_CLOSED       =  0.8;


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
        Trajectory fourRingsA = drive.trajectoryBuilder(startPose)//to launch line
                .lineTo(new Vector2d(-12,-24))
                .build();
        Trajectory fourRingsB = drive.trajectoryBuilder(fourRingsA.end())//launch rings
                .strafeTo(new Vector2d(-10, -31))
                .build();
        Trajectory fourRingsC = drive.trajectoryBuilder(fourRingsB.end())//drop wobble goal
//                .splineToSplineHeading(new Pose2d(54,-36, Math.toRadians(0)), Math.toRadians(180))
                .lineToSplineHeading(new Pose2d(56,-46, Math.toRadians(180)))
                .build();
        Trajectory fourRingsD = drive.trajectoryBuilder(fourRingsC.end())//ring stack
                .lineToSplineHeading(new Pose2d(0, -52, Math.toRadians(90)))
                .build();
        Trajectory fourRingsE = drive.trajectoryBuilder(fourRingsD.end())//second wobble
                .lineToConstantHeading(new Vector2d(-48, -48))
                .build();
        Trajectory fourRingsF = drive.trajectoryBuilder(fourRingsE.end())//ring stack
                .lineToConstantHeading(new Vector2d(0, -52))
                .build();
        Trajectory fourRingsG = drive.trajectoryBuilder(fourRingsF.end())// drop wobble
                .lineToSplineHeading(new Pose2d(48, -48, Math.toRadians(180)))
                .build();
        Trajectory fourRingsH = drive.trajectoryBuilder(fourRingsG.end()) //park
                .lineTo(new Vector2d(12, -48))
                .build();

        //TODO add auto code here

        drive.followTrajectory(fourRingsA);
        drive.followTrajectory(fourRingsB); //launch rings
//        rpm.setRPM(2560);//launcher wheel rev up
//        launchRings();
//        sleep(5000);
//        trigger.setPosition(.8);//set to launch
//        sleep(500);
//        trigger.setPosition(0.1);//launch
//        sleep(500);
//        trigger.setPosition(.9);//set to launch
//        sleep(500);
//        trigger.setPosition(0.1);//launch
//        sleep(500);
//        trigger.setPosition(.9);//set to launch
//        sleep(500);
//        trigger.setPosition(0.1);//launcher
//        sleep(500);
//        trigger.setPosition(.9);//set to launch
//        sleep(500);
//        rpm.setRPM(0);
        drive.followTrajectory(fourRingsC);//wobble goal 1
//        armPower(-.5, 2.3); //arm down
//        sleep(500);
//        grabber(GRABBER_OPEN);
        drive.followTrajectory(fourRingsD);//ring stack
        drive.followTrajectory(fourRingsE);//second wobble
//        sleep(500);
//        grabber(GRABBER_CLOSED);
//        sleep(1000);
//        armPower(.5,1);//arm up
        drive.followTrajectory(fourRingsF);//drop second wobble
        drive.followTrajectory(fourRingsG);
//        armPower(-.5, 1);//arm down
//        sleep(500);
//        grabber(GRABBER_OPEN);
        drive.followTrajectory(fourRingsH);
//        sleep(2000);

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
        sleep(3000);
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
}

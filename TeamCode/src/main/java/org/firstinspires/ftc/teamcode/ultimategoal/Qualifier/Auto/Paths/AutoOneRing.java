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
public class AutoOneRing extends LinearOpMode {
    private Servo trigger;
    private Servo grabber;
    private DcMotorEx arm;
    private DcMotor intake;
    private ElapsedTime runtime = new ElapsedTime();
    public static final double GRABBER_OPEN       =  0.1;
    public static final double GRABBER_CLOSED       =  0.9;


    @Override
    public void runOpMode() throws InterruptedException {
        // Declare your drive class
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        DcMotorEx launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        arm    = hardwareMap.get(DcMotorEx.class, "arm");
        intake = hardwareMap.get(DcMotor.class,"intake");

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

        while (opModeIsActive()) {
            telemetry.addData("RPM", rpm.getRPM());
            telemetry.update();
        }

        // Example spline path from SplineTest.java
        // Make sure the start pose matches with the localizer's start pose
        Trajectory oneRingA = drive.trajectoryBuilder(startPose)//to launch line
                .lineTo(new Vector2d(-12,-24))
                .build();
        Trajectory oneRingB = drive.trajectoryBuilder(oneRingA.end())//launch rings
                .strafeTo(new Vector2d(-10, -31))
                .build();
        Trajectory oneRingC = drive.trajectoryBuilder(oneRingB.end())
                .lineToSplineHeading(new Pose2d(22,-32, Math.toRadians(-90))) //drop wobble goal
                .build();
        Trajectory oneRingD = drive.trajectoryBuilder(oneRingC.end())//ring stack
                .lineToSplineHeading(new Pose2d(-28, -24, Math.toRadians(180)))
                .build();
        Trajectory oneRingE = drive.trajectoryBuilder(oneRingD.end())//
                .lineToConstantHeading(new Vector2d(-41, -38))
                .build();
        Trajectory oneRingF = drive.trajectoryBuilder(oneRingE.end())// second wobble
                .lineToConstantHeading(new Vector2d(-43, -38))
                .build();
        Trajectory oneRingG = drive.trajectoryBuilder(oneRingF.end()) //drop wobble
                .lineToSplineHeading(new Pose2d(16, -40, Math.toRadians(-90)))
                .build();
        Trajectory oneRingH = drive.trajectoryBuilder(oneRingG.end()) //park
                .lineTo(new Vector2d(12, -40))
                .build();

        //TODO add auto code here
        rpm.setRPM(2560);//launcher wheel rev up
//        sleep(10000);
//        drive.followTrajectory(oneRingA);
//        drive.followTrajectory(oneRingB); //launch rings
//        launchRings();
        sleep(3000);
//        trigger.setPosition(.8);//set to launch
//        sleep(500);
//        trigger.setPosition(0.1);//launch
//        sleep(550);
//        trigger.setPosition(.9);//set to launch
//        sleep(250);
//        trigger.setPosition(0.1);//launch
//        sleep(550);
//        trigger.setPosition(.9);//set to launch
//        sleep(250);
//        trigger.setPosition(0.1);//launcher
//        sleep(550);
//        trigger.setPosition(.9);//set to launch
//        sleep(250);
//        drive.followTrajectory(oneRingC);//wobble goal 1
//        armPower(-.5, 2.3); //arm down
//        sleep(500);
//        grabber(GRABBER_OPEN);
//        armPower(.5,1);//arm up
//        drive.followTrajectory(oneRingD);//ring stack
//        drive.followTrajectory(oneRingE);
//        sleep(500);
//        armPower(-.5, 1.2);//arm down
//        drive.followTrajectory(oneRingF);//second wobble
//        sleep(1000);
//        grabber(GRABBER_CLOSED);
//        sleep(1000);
//        armPower(.5,1);//arm up
//        drive.followTrajectory(oneRingG);//drop second wobble
//        armPower(-.5, 1);//arm down
//        sleep(500);
//        grabber(GRABBER_OPEN);
//        sleep(500);
//        armPower(.5,1);//arm up
//        drive.followTrajectory(oneRingH);//park
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
    public void intakePower(double power){
        intake.setPower(power);
    }
}

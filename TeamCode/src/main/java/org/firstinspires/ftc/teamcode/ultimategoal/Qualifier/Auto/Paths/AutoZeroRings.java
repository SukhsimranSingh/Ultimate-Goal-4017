package org.firstinspires.ftc.teamcode.ultimategoal.Qualifier.Auto.Paths;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
public class AutoZeroRings extends LinearOpMode {
    private Servo trigger;
    private Servo grabber;
    private DcMotorEx arm;
    private ElapsedTime runtime = new ElapsedTime();
    public static final double GRABBER_OPEN       =  0.1;
    public static final double GRABBER_CLOSED       =  1.0;


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
        while (opModeIsActive()) {
            telemetry.addData("RPM", rpm.getRPM());
            telemetry.update();
        }

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

        Trajectory zeroRingG = drive.trajectoryBuilder(zeroRingD.end())//lining up
                .strafeTo(new Vector2d(6,-38))
                .build();

        Trajectory zeroRingH = drive.trajectoryBuilder(zeroRingE.end())// second wobble
                .lineToConstantHeading(new Vector2d(-48, -37))
                .build();

        Trajectory zeroRingI = drive.trajectoryBuilder(zeroRingF.end()) //drop wobble and park
                .lineTo(new Vector2d(12, -36))
                .build();

        //TODO add auto code here

        rpm.setRPM(2255);//launcher wheel rev up
        drive.followTrajectory(zeroRingA);
        drive.followTrajectory(zeroRingB); //launch rings
        sleep(550);
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
        rpm.setRPM(0);
        drive.followTrajectory(zeroRingE);//wobble goal 1
        armPower(-.5, 2.3); //arm down
        sleep(500);
        grabber(GRABBER_OPEN);
        drive.followTrajectory(zeroRingF);
        drive.followTrajectory(zeroRingG);
        drive.followTrajectory(zeroRingH);
        sleep(500);
        grabber(GRABBER_CLOSED);
        sleep(1000);
        armPower(.5,1);//arm up
        drive.followTrajectory(zeroRingI);
        armPower(-.5, .75);//arm down
        sleep(500);
        grabber(GRABBER_OPEN);
        armPower(.5,1);//arm up
        sleep(2000);



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
        trigger.setPosition(.8);//set to launch
        sleep(500);
        trigger.setPosition(0.1);//launch
        sleep(500);
        trigger.setPosition(.8);//set to launch
        sleep(500);
        trigger.setPosition(0.1);//launcher
        sleep(500);
        trigger.setPosition(.8);//set to launch
        sleep(500);
    }
}

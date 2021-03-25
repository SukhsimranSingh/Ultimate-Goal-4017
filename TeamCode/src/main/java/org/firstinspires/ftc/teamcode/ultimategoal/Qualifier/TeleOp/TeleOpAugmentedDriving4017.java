package org.firstinspires.ftc.teamcode.ultimategoal.Qualifier.TeleOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.advanced.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.ultimategoal.Qualifier.util.PoseStorage;
import org.firstinspires.ftc.teamcode.ultimategoal.Qualifier.util.RPMTool;

/**
 * This opmode demonstrates how one can augment driver control by following Road Runner arbitrary
 * Road Runner trajectories at any time during teleop. This really isn't recommended at all. This is
 * not what Trajectories are meant for. A path follower is more suited for this scenario. This
 * sample primarily serves as a demo showcasing Road Runner's capabilities.
 * <p>
 * This bot starts in driver controlled mode by default. The player is able to drive the bot around
 * like any teleop opmode. However, if one of the select buttons are pressed, the bot will switch
 * to automatic control and run to specified location on its own.
 * <p>
 * If A is pressed, the bot will generate a splineTo() trajectory on the fly and follow it to
 * targetA (x: 45, y: 45, heading: 90deg).
 * <p>
 * If B is pressed, the bot will generate a lineTo() trajectory on the fly and follow it to
 * targetB (x: -15, y: 25, heading: whatever the heading is when you press B).
 * <p>
 * If Y is pressed, the bot will turn to face 45 degrees, no matter its position on the field.
 * <p>
 * Pressing X will cancel trajectory following and switch control to the driver. The bot will also
 * cede control to the driver once trajectory following is done.
 * <p>
 * The following may be a little off with this method as the trajectory follower and turn
 * function assume the bot starts at rest.
 * <p>
 * This sample utilizes the SampleMecanumDriveCancelable.java class.
 */
@TeleOp(group = "advanced")
public class TeleOpAugmentedDriving4017 extends LinearOpMode {
    // Define 2 states, drive control or automatic control
    enum Mode {
        DRIVER_CONTROL,
        AUTOMATIC_CONTROL
    }

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;
    private DcMotor intake;
    private DcMotor arm;

    public static final double GRABBER_OPEN       =  0.2 ;
    public static final double GRABBER_CLOSED     =  0.9 ;
    public static final double TRIGGER_PRESSED    =  0.1 ;
    public static final double TRIGGER_UNPRESSED  =  0.8 ;

    private Servo grabber;
    private Servo trigger;

    Mode currentMode = Mode.DRIVER_CONTROL;

    // The coordinates we want the bot to automatically go to when we press the A button
    Vector2d targetAVector = new Vector2d(-10, -31);
    // The heading we want the bot to end on for targetA
    double targetAHeading = Math.toRadians(0);

    // The location we want the bot to automatically go to when we press the B button
    Vector2d targetBVector = new Vector2d(-10, -31);

    // The angle we want to align to when we press Y
    double targetAngle = Math.toRadians(10);

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize custom cancelable SampleMecanumDrive class
        SampleMecanumDriveCancelable drive = new SampleMecanumDriveCancelable(hardwareMap);

        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
        drive.setPoseEstimate(PoseStorage.currentPose);
        DcMotorEx launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        launcher.setDirection(DcMotorSimple.Direction.FORWARD);

        RPMTool rpm = new RPMTool(launcher, 28);
        //Gobilda 6000 rpm motor

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        grabber = hardwareMap.servo.get("grabber");
        trigger = hardwareMap.servo.get("trigger");

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            // Update the drive class
            drive.update();

            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            // Print pose to telemetry
            telemetry.addData("mode", currentMode);
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("RPM", rpm.getRPM());
            telemetry.update();

            // We follow different logic based on whether we are in manual driver control or switch
            // control to the automatic mode
            switch (currentMode) {
                case DRIVER_CONTROL:
                    drive.setWeightedDrivePower(
                            new Pose2d(
                                    -gamepad1.left_stick_y,
                                    -gamepad1.left_stick_x,
                                    -gamepad1.right_stick_x
                            )
                    );

                    if (gamepad1.a) {
                        // If the A button is pressed on gamepad1, we generate a splineTo()
                        // trajectory on the fly and follow it
                        // We switch the state to AUTOMATIC_CONTROL

                        Trajectory traj1 = drive.trajectoryBuilder(poseEstimate)
                                .splineTo(targetAVector, targetAHeading)
                                .build();

                        drive.followTrajectoryAsync(traj1);

                        currentMode = Mode.AUTOMATIC_CONTROL;
                    } else if (gamepad1.b) {
                        // If the B button is pressed on gamepad1, we generate a lineTo()
                        // trajectory on the fly and follow it
                        // We switch the state to AUTOMATIC_CONTROL
                        Trajectory fourRingB = drive.trajectoryBuilder(poseEstimate)//launch rings
                                .strafeTo(new Vector2d(-3, -16))
                                .build();
                        Trajectory fourRingC = drive.trajectoryBuilder(fourRingB.end())//launch rings
                                .strafeTo(new Vector2d(-3, -8))
                                .build();
                        Trajectory fourRingD = drive.trajectoryBuilder(fourRingC.end())//launch rings
                                .strafeTo(new Vector2d(-3, -2))
                                .build();

                        rpm.setRPM(2259);//launcher wheel rev up
                        drive.followTrajectory(fourRingB); //launch rings
                        sleep(750);
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

//                        Trajectory traj1 = drive.trajectoryBuilder(poseEstimate)
//                                .lineTo(targetBVector)
//                                .build();

//                        drive.followTrajectoryAsync(traj1);

                        currentMode = Mode.AUTOMATIC_CONTROL;
                    } else if (gamepad1.y) {
                        // If Y is pressed, we turn the bot to the specified angle to reach
                        // targetAngle (by default, 30 degrees)

                        drive.turnAsync(Angle.normDelta(targetAngle - poseEstimate.getHeading()));

                        currentMode = Mode.AUTOMATIC_CONTROL;
                    }
                    if (gamepad1.left_bumper){
                        rpm.setRPM(2560);
                    }
                    if (gamepad1.right_bumper){
                        rpm.setRPM(2700);
                    }
                    if (gamepad1.dpad_up){
                        rpm.setRPM(2259);
                    }
                    if (gamepad1.dpad_down){
                        launcher.setPower(0);
                    }
                    //trigger
                    if (gamepad1.right_trigger > 0){
                        trigger.setPosition(TRIGGER_PRESSED);
                    }
                    else {
                        trigger.setPosition(TRIGGER_UNPRESSED);
                    }

                    //Intake
                    double intakePower = gamepad1.left_trigger;
                    intake.setPower(intakePower);

                    //ARM
                    arm.setPower(-gamepad2.right_stick_y);
                    //Grabber
                    if (gamepad2.left_trigger > 0) {
                        grabber.setPosition(GRABBER_OPEN);
                    }
                    else {
                        grabber.setPosition(GRABBER_CLOSED);
                    }

                    break;
                case AUTOMATIC_CONTROL:
                    // If x is pressed, we break out of the automatic following
                    if (gamepad1.x) {
                        drive.cancelFollowing();
                        currentMode = Mode.DRIVER_CONTROL;
                    }

                    // If drive finishes its task, cede control to the driver
                    if (!drive.isBusy()) {
                        currentMode = Mode.DRIVER_CONTROL;
                    }
                    break;
            }
        }
    }
}

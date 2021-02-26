package org.firstinspires.ftc.teamcode.ultimategoal.scrimmage.TeleOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.advanced.PoseStorage;
import org.firstinspires.ftc.teamcode.ultimategoal.scrimmage.util.RPMTool;

/**
 * This opmode demonstrates how to create a teleop using just the SampleMecanumDrive class without
 * the need for an external robot class. This will allow you to do some cool things like
 * incorporating live trajectory following in your teleop. Check out TeleOpAgumentedDriving.java for
 * an example of such behavior.
 * <p>
 * This opmode is essentially just LocalizationTest.java with a few additions and comments.
 */
@Disabled

@TeleOp(group = "advanced")
public class NonAugmentedTeleOp extends LinearOpMode {

    Hardware2 robot           = new Hardware2();   // Use our hardware
    RPMTool launcher = new RPMTool(robot.launcher, 28);
    public static final double highGoalRPM = 2500;
    public static final double powershotRPM = 2375;
    public static final double GRABBER_OPEN       =  0.2 ;
    public static final double GRABBER_CLOSED       =  0.8 ;
    public static final double TRIGGER_PRESSED       =  0.5 ;
    public static final double TRIGGER_UNPRESSED       =  0.8 ;
    private ElapsedTime runtime = new ElapsedTime();



    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize SampleMecanumDrive
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        robot.init(hardwareMap);


        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
        drive.setPoseEstimate(PoseStorage.currentPose);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );
            // Update everything. Odometry. Etc.
            drive.update();

            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            //launcher speed control
            if (gamepad1.left_bumper){
                launcher.setRPM(powershotRPM);
            }

            if (gamepad1.right_bumper){
                launcher.setRPM(highGoalRPM);
            }

            if (gamepad1.dpad_up){
                launcher.setRPM(2480);
            }

            if (gamepad1.dpad_down){
                robot.launcher.setPower(0);
            }

            //trigger
            if (gamepad1.right_trigger > 0){
                robot.trigger.setPosition(TRIGGER_PRESSED);
                robot.trigger.setPosition(TRIGGER_UNPRESSED);
            }
            else {
                robot.trigger.setPosition(TRIGGER_UNPRESSED);
            }


            //ARM
            if (gamepad2.right_stick_y > 0) {
                armPower(.5);
            }
            else if (gamepad2.right_stick_y < 0){
                armPower(-.5);
            }
            else {
                armPower(0);
            }

            //Grabber
            if (gamepad2.left_trigger > 0) {
                grabber(GRABBER_OPEN);
            }
            else if (gamepad2.right_trigger > 0){
                grabber(GRABBER_CLOSED);
            }
            else {
                //do nothing
            }
            //Intake
            if (gamepad1.left_trigger > 0){
                intake(.5);
            }
            else {
                intake(0);
            }



            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }
    public void armPower(double power){
            robot.arm.setPower(power);
    }

    public void grabber(double position){
        robot.grabber.setPosition(position);

    }

    public void intake(double power){
        robot.intake.setPower(power);
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

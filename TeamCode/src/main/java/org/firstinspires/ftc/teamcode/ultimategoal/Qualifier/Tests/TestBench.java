package org.firstinspires.ftc.teamcode.ultimategoal.Qualifier.Tests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ultimategoal.Qualifier.util.MecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.ultimategoal.Qualifier.util.PoseStorage;
import org.firstinspires.ftc.teamcode.ultimategoal.Qualifier.util.RPMTool;
@Disabled
@TeleOp(name="TestBench")
public class TestBench extends LinearOpMode {

    //Declare OpMode members
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;
    private DcMotor intake;
    private DcMotor arm;

    private Servo grabber;
    private Servo trigger;

    int ARM_UP = -40;
    int ARM_DOWN = 40;

    private boolean is1YPressed = false;
    private boolean slowDrive = false;
    private double servoPosition = 0.16;



    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        MecanumDriveCancelable drive = new MecanumDriveCancelable(hardwareMap);
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
        runtime.reset();

        while (opModeIsActive()) {
            // Update the drive class
            drive.update();

            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();

            double v0 = -1*gamepad1.right_stick_y + -1*gamepad1.right_stick_x +(1*gamepad1.left_stick_x);
            double v1 = -1*gamepad1.right_stick_y + 1*gamepad1.right_stick_x + (1*gamepad1.left_stick_x);
            double v2 = -1*gamepad1.right_stick_y + -1*gamepad1.right_stick_x +(-1*gamepad1.left_stick_x);
            double v3 = -1*gamepad1.right_stick_y + 1*gamepad1.right_stick_x +(-1*gamepad1.left_stick_x);
            leftFront.setPower(v0);
            leftRear.setPower(v1);
            rightFront.setPower(v2);
            rightRear.setPower(v3);

            //launcher speed control
            if (gamepad1.left_bumper){
                rpm.setRPM(2350);
            }

            if (gamepad1.right_bumper){
                rpm.setRPM(2500);
            }

            if (gamepad1.dpad_up){
                rpm.setRPM(2480);
            }

            if (gamepad1.dpad_down){
                launcher.setPower(0);
            }

            //trigger
            if (gamepad1.right_trigger > 0){
            trigger.setPosition(.3);
            }
            else {
                trigger.setPosition(.8);
            }


            //ARM
            arm.setPower(-gamepad2.right_stick_y);

            //Grabber
            if (gamepad2.left_trigger > 0) {
                grabber.setPosition(.4);
            }
            else {
                grabber.setPosition(.7);
            }

            //Intake
            double intakePower = gamepad2.left_stick_y;
            intake.setPower(intakePower);

            if (gamepad2.dpad_down){
                armPosition(ARM_DOWN);
            }
            if (gamepad2.dpad_up){
                armPosition(ARM_UP);

            }

        }

    }
    public void armPosition (int angle){
        int ticks = arm.getCurrentPosition();
        int ticksPerRevInput = 28;
        int gearRatio = 256;
        int ticksPerRevOutput = ticksPerRevInput*gearRatio;
        int ticksPerAngle = (int) (3.141592654/ticksPerRevOutput);
        int target = angle * ticksPerAngle;

        arm.setTargetPosition(ticks + target);


    }
}
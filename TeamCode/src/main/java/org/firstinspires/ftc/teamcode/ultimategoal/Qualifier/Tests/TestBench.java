package org.firstinspires.ftc.teamcode.ultimategoal.Qualifier.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ultimategoal.Qualifier.util.RPMTool;

@TeleOp(name="TestBench")
public class TestBench extends LinearOpMode {

    //Declare OpMode members
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx motor1 = null;
    private DcMotorEx motor2 = null;
    private DcMotorEx motor3 = null;
    private DcMotorEx motor4 = null;
    private DcMotorSimple simpleMotor1 = null;
    private DcMotorSimple simpleMotor2 = null;
    private Servo Servo;
    private CRServo CRservo = null;
    DigitalChannel limitSwitch;


    private boolean is1YPressed = false;
    private boolean slowDrive = false;
    private double servoPosition = 0.16;

    RPMTool rpm = new RPMTool(motor1, 28);
    //Gobilda 6000 rpm motor

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        motor1 = hardwareMap.get(DcMotorEx.class, "motor1");
        motor2 = hardwareMap.get(DcMotorEx.class, "motor2");
        motor3 = hardwareMap.get(DcMotorEx.class, "motor3");
        motor4 = hardwareMap.get(DcMotorEx.class, "motor4");
        simpleMotor1 = hardwareMap.get(DcMotorSimple.class, "simpleMotor1");
        simpleMotor2 = hardwareMap.get(DcMotorSimple.class, "simpleMotor2");
        Servo = hardwareMap.servo.get("Servo");
        CRservo = hardwareMap.get(CRServo.class, "CRservo");
        limitSwitch = hardwareMap.get(DigitalChannel.class, "limitSwitch");

        Servo.setPosition(servoPosition);
        limitSwitch.setMode(DigitalChannel.Mode.INPUT);

        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            double mp1;
            double mp2;
            double mp3;
            double mp4;
            double smp1 = gamepad1.left_stick_x;
            double smp2 = gamepad2.right_stick_y;

            if (gamepad1.y) {
                if (!is1YPressed) {
                    is1YPressed = true;
                    slowDrive = !slowDrive;
                } else {
                    is1YPressed = false;
                }
            }

            if (gamepad2.left_trigger > 0) {                                                        //Capstone Servo Control
                servoPosition = gamepad2.left_trigger;
//                servoPosition = Range.clip(servoPosition, .16, .75);
            } else {
                servoPosition = 0.16;
            }
            Servo.setPosition(servoPosition);
            telemetry.addData("Servo pos: %s", Servo.getPosition());
            telemetry.update();

            if (gamepad2.right_stick_y > 0) {
                simpleMotor2.setPower(smp2);
            }
            else if (gamepad2.right_stick_y < 0) {
                simpleMotor2.setPower(smp2);
            }
            else {
                simpleMotor2.setPower(0);

            }


        }
    }
}
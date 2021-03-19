package org.firstinspires.ftc.teamcode.ultimategoal.Qualifier.Tests;

import org.firstinspires.ftc.teamcode.ultimategoal.Qualifier.util.RPMTool;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
@Config
@TeleOp(name = "rpmTest")
public class RPMTest extends LinearOpMode {
    public static final double TRIGGER_PRESSED    =  0.1 ;
    public static final double TRIGGER_UNPRESSED  =  0.8 ;
    public static double RPM = 2500;
    private Servo trigger;


    public void runOpMode(){

        DcMotorEx motor1 = hardwareMap.get(DcMotorEx.class, "launcher");
        motor1.setDirection(DcMotorSimple.Direction.FORWARD);
        trigger = hardwareMap.servo.get("trigger");


        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        RPMTool rpm = new RPMTool(motor1, 28);
        //Gobilda 6000 rpm motor

        waitForStart();

        while(!isStopRequested()){

            if (gamepad1.a){
                rpm.setRPM(RPM);
            }

            if (gamepad1.b){

                rpm.setRPM(3000);
            }

            if (gamepad1.dpad_up){
                rpm.setRPM(2250);
            }

            if (gamepad1.x){
                motor1.setPower(0);
            }
            //trigger
            if (gamepad1.right_trigger > 0){
                trigger.setPosition(TRIGGER_PRESSED);
            }
            else {
                trigger.setPosition(TRIGGER_UNPRESSED);
            }

            telemetry.addData(" Current rpm", rpm.getRPM());
            telemetry.addData("expected rpm", 2500);
            telemetry.update();

        }

    }

}
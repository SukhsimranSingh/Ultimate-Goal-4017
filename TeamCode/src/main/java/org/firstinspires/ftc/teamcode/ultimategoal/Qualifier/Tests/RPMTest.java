package org.firstinspires.ftc.teamcode.ultimategoal.Qualifier.Tests;

import org.firstinspires.ftc.teamcode.ultimategoal.Qualifier.util.RPMTool;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "rpmTest")
public class RPMTest extends LinearOpMode {

    public void runOpMode(){

        DcMotorEx motor1 = hardwareMap.get(DcMotorEx.class, "motor1");

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        RPMTool rpm = new RPMTool(motor1, 28);
        //Gobilda 6000 rpm motor

        waitForStart();

        while(!isStopRequested()){

            if (gamepad1.a){
                rpm.setRPM(2375);
            }

            if (gamepad1.b){

                rpm.setRPM(2480);
            }

            if (gamepad1.dpad_up){
                rpm.setRPM(2510);
            }

            if (gamepad1.x){
                motor1.setPower(0);
            }
            if (gamepad1.left_trigger > 0) {                                                        //Capstone Servo Control
                double mP1 = gamepad1.left_trigger * 5300;
                motor1.setPower(mP1);
            }

            telemetry.addData(" rpm", rpm.getRPM());
            telemetry.update();

        }

    }

}
package org.firstinspires.ftc.teamcode.ultimategoal.Qualifier.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class IntakeTest extends LinearOpMode {
    public void runOpMode(){

        DcMotorEx motor1 = hardwareMap.get(DcMotorEx.class, "motor1");


        waitForStart();
        while(!isStopRequested()){

            double intakeP = gamepad1.right_stick_y;

            motor1.setPower(intakeP);

            telemetry.addData(" Current power", intakeP);
        }

        }
}


package org.firstinspires.ftc.teamcode.ultimategoal.Qualifier.Auto.Subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.ultimategoal.Qualifier.util.Hardware;
import org.firstinspires.ftc.teamcode.ultimategoal.Qualifier.util.MecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.ultimategoal.Qualifier.util.PoseStorage;
import org.firstinspires.ftc.teamcode.ultimategoal.Qualifier.util.RPMTool;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvPipeline;

@Autonomous
public class AutonomousPath extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    MecanumDriveCancelable drive;

    Pose2d startPose = new Pose2d(-63, -24, Math.toRadians(0));//side one
//    Pose2d startPose = new Pose2d(-63,-48, Math.toRadians(180));//side two

    Trajectory zeroRings = drive.trajectoryBuilder(startPose)
            .lineTo(new Vector2d(0,-24)) //shoot rings
            .strafeTo(new Vector2d(6,-50)) //drop wobble goal
            .strafeTo(new Vector2d(6,-36)) //lineup to rings
            .lineToConstantHeading(new Vector2d(-48, -36)) //grab 2nd wobble goal
            .lineTo(new Vector2d(12, -36))//lineup to park
            .strafeTo(new Vector2d(12, -50))//drop 2nd wobble
            .build();
    Trajectory oneRing = drive.trajectoryBuilder(startPose)
            .lineTo(new Vector2d(0,-24))//shoot rings
            .lineTo(new Vector2d(24,-24))//drop wobble goal
            .splineToSplineHeading(new Pose2d(-8,-36, Math.toRadians(0)), Math.toRadians(26))//pickup rings
            .lineToConstantHeading(new Vector2d(-48, -36)) //grab 2nd wobble goal
            .lineToSplineHeading(new Pose2d(24,-36, Math.toRadians(90)))//drop wobble goal
            .strafeTo(new Vector2d(0,-36))//park
            .build();
    Trajectory fourRings = drive.trajectoryBuilder(startPose)
            .lineTo(new Vector2d(0,-24))//shoot rings
            .lineTo(new Vector2d(54,-48))//drop wobble goal
            .splineToSplineHeading(new Pose2d(-8,-36, Math.toRadians(0)), Math.toRadians(26))//pickup rings
            .lineToConstantHeading(new Vector2d(-48, -36)) //grab 2nd wobble goal
            .lineToConstantHeading(new Vector2d(48,-40))//drop wobble goal
            .lineToConstantHeading(new Vector2d(0,-40))//park
            .build();


    @Override
    public void runOpMode() throws InterruptedException {
        drive = new MecanumDriveCancelable(hardwareMap);


        waitForStart();
    }





}
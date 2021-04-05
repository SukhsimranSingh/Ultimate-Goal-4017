package org.firstinspires.ftc.teamcode.ultimategoal.vision.HighGoalDistance;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvPipeline;

@TeleOp
public class GoalDistanceDetector extends LinearOpMode
{
    OpenCvCamera webcam;
    UGAdvancedHighGoalPipeline pipeline;
    double fov = 52;//this an angle in degrees

    @Override
    public void runOpMode()
    {
//        int cameraMonitorViewId = hardwareMap.appContext
//                .getResources()
//                .getIdentifier(
//                        "cameraMonitorViewId",
//                        "id",
//                        hardwareMap.appContext.getPackageName()
//                ); // for camera preview
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"));
        FtcDashboard.getInstance().startCameraStream(webcam, 0);
        pipeline = new UGAdvancedHighGoalPipeline(fov,3);
        webcam.setPipeline(pipeline);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

//        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
//        {
//            @Override
//            public void onOpened()
//            {
//                webcam.startStreaming(320,240,  OpenCvCameraRotation.UPRIGHT);
//            }
//        });

        waitForStart();

        while (opModeIsActive())
        {
            telemetry.addData("Distance from wall", pipeline.getDistanceToGoalWall(UGAngleHighGoalPipeline.Target.RED));
            telemetry.addData("Distance from high goal", pipeline.getDistanceToGoal(UGAngleHighGoalPipeline.Target.RED));
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }
    }

}
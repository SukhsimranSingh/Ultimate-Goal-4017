package org.firstinspires.ftc.teamcode.ultimategoal.vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.ultimategoal.Qualifier.TeleOp.TeleOpAugmentedDriving4017;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

/**
 * In this sample, we demonstrate how to use the advanced features provided
 * by the {@link OpenCvInternalCamera} interface
 */
@TeleOp(name = "OpenCVringdistance", group = "AI")
public class DistanceFromRing extends LinearOpMode {
  OpenCvCamera webcam;
  DistanceFromRingTracker distanceTracker;

  @Override
  public void runOpMode() {
    distanceTracker = new DistanceFromRingTracker(
      0.5
    );
    webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"));
    FtcDashboard.getInstance().startCameraStream(webcam, 0);
    webcam.setPipeline(distanceTracker);

    // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
    // out when the RC activity is in portrait. We do our actual image processing assuming
    // landscape orientation, though.
    webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

    webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
    {
      @Override
      public void onOpened()
      {
        webcam.startStreaming(320,240,  OpenCvCameraRotation.UPRIGHT);
      }
    });
    telemetry = FtcDashboard.getInstance().getTelemetry();
    waitForStart();

    while (opModeIsActive() && !isStopRequested()) {
      telemetry.addData("FPS", webcam.getFps());
      telemetry.addData("Pipeline (ms)",webcam.getPipelineTimeMs());
      telemetry.addData(
        "Total Frame time (ms)",
        webcam.getTotalFrameTimeMs()
      );
      telemetry.addData(
        "W, H (px)",
        distanceTracker.bounds.size.width +
        ", " +
        distanceTracker.bounds.size.height
      );
      telemetry.addData("distance (in)", distanceTracker.computeDistance());
      telemetry.addData("focalLength", distanceTracker.computefocalLength());
      telemetry.update();
      sleep(100);
    }
  }
}

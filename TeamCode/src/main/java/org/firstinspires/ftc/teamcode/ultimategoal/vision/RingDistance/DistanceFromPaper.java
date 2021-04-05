package org.firstinspires.ftc.teamcode.ultimategoal.vision.RingDistance;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.*;

import org.openftc.easyopencv.*;

/**
 * In this sample, we demonstrate how to use the advanced features provided
 * by the {@link OpenCvInternalCamera} interface
 */
@Disabled

@TeleOp(name = "OpenCV Paper Detection Demo Algorithm", group = "AI")
public class DistanceFromPaper extends LinearOpMode {

  @Override
  public void runOpMode() {
    DistanceFromPaperTracker distanceTracker = new DistanceFromPaperTracker(
      0.5
    );
    OpenCVPipelineRunner runner = new OpenCVPipelineRunner(
      hardwareMap,
      distanceTracker
    );

    runner.start();
    FtcDashboard.getInstance().startCameraStream(runner.webcam, 0);
    telemetry = FtcDashboard.getInstance().getTelemetry();
    waitForStart();

    while (opModeIsActive()) {
      telemetry.addData("FPS", runner.webcam.getFps());
      telemetry.addData("Pipeline (ms)", runner.webcam.getPipelineTimeMs());
      telemetry.addData(
        "Total Frame time (ms)",
        runner.webcam.getTotalFrameTimeMs()
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

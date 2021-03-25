package org.firstinspires.ftc.teamcode.ultimategoal.vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ultimategoal.Qualifier.TeleOp.TeleOpAugmentedDriving4017;
import org.openftc.easyopencv.OpenCvInternalCamera;

/**
 * In this sample, we demonstrate how to use the advanced features provided
 * by the {@link OpenCvInternalCamera} interface
 */
@TeleOp(name = "OpenCVringdistance", group = "AI")
public class DistanceFromRing extends LinearOpMode {


  @Override
  public void runOpMode() {
    DistanceFromRingTracker distanceTracker = new DistanceFromRingTracker(
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

    while (opModeIsActive() && !isStopRequested()) {
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

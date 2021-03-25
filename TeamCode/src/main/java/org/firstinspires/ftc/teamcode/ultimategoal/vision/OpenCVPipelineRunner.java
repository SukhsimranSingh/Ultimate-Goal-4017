package org.firstinspires.ftc.teamcode.ultimategoal.vision;

import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.*;

public class OpenCVPipelineRunner {

  private OpenCvTrackerApiPipeline openCvTrackerApiPipeline;
  /**
   * The phonecam module @see org.openftc.easyopencv.OpenCvInternalCamera
   */
  public OpenCvCamera webcam;

  OpenCVPipelineRunner(HardwareMap hardwareMap, OpenCvTracker... trackers) {
    int cameraMonitorViewId = hardwareMap.appContext
      .getResources()
      .getIdentifier(
        "cameraMonitorViewId",
        "id",
        hardwareMap.appContext.getPackageName()
      ); // for camera preview
    webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"));

    webcam.openCameraDevice();
    openCvTrackerApiPipeline = new OpenCvTrackerApiPipeline();
    webcam.setPipeline(openCvTrackerApiPipeline);

    // Set the viewport renderer to use the gpu so we have better handling
    webcam.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);

    for (OpenCvTracker tracker : trackers) {
      openCvTrackerApiPipeline.addTracker(tracker);
    }
  }

  public void start(boolean showData) {
    webcam.showFpsMeterOnViewport(showData);
    /*
     * We use the most verbose version of #startStreaming(), which allows us to specify whether we want to use double
     * (default) or single buffering. See the JavaDoc for this method for more details
     */
    webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

  }

  public void start() {
    start(false);
  }

  public void addTracker(OpenCvTracker tracker) {
    openCvTrackerApiPipeline.addTracker(tracker);
  }

  public void removeTracker(OpenCvTracker tracker) {
    openCvTrackerApiPipeline.removeTracker(tracker);
  }
}

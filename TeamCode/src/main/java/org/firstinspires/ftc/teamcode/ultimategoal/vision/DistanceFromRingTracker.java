package org.firstinspires.ftc.teamcode.ultimategoal.vision;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvTracker;

import java.util.ArrayList;

@Config
public class DistanceFromRingTracker extends OpenCvPipeline {

  /**
   * The hand pre-calculated value of the focal length for calculating the distance using @see #computeDistance()
   * This value can be calculated with <code>F = (P x  D) / W</code> f=focalLength P=perceivedPxCount W=actual width
   */
  public static double distance = 0.0;
  public static double focalLength = 642.9090909;

  private final Mat output = new Mat();
  private final Mat gray = new Mat();
  private final Mat threshold = new Mat();
  private final Mat edges = new Mat();
  private int thresholdThresh = 200;
  private int thresholdMaxVal = 255;
  private int cannyMinThreshold = 120;
  private int cannyMaxThreshold = 150;
  private final double pctBoundsThreshold;
  private final ArrayList<MatOfPoint> contours = new ArrayList<>();
  private final MatOfPoint2f largestContour2f = new MatOfPoint2f();

  /**
   * The bounds of the detected piece of ring
   */
  public RotatedRect bounds = new RotatedRect();

  /**
   * Class Constructor for the ring distance detection pipeline
   *
   * @param pctThreshold percent threshold for determining false positives
   */
  DistanceFromRingTracker(double pctThreshold) {
    pctBoundsThreshold = pctThreshold;
  }

  /**
   * Class Constructor for the ring distance detection pipeline
   *
   * @param pctThreshold percent threshold for determining false positives
   * @param thresholdThresh threshold for the threshold step of the pipeline
   * @param thresholdMaxVal threshold maximum value for the threshold step of the pipeline
   */
  public DistanceFromRingTracker(
    double pctThreshold,
    int thresholdThresh,
    int thresholdMaxVal
  ) {
    this.pctBoundsThreshold = pctThreshold;
    this.thresholdThresh = thresholdThresh;
    this.thresholdMaxVal = thresholdMaxVal;
  }

  /**
   * Class Constructor for ring distance detection pipeline
   *
   * @param pctThreshold percent threshold for determining false positives
   * @param thresholdThresh threshold for the threshold step of the pipeline
   * @param thresholdMaxVal threshold maximum value for the threshold step of the pipeline
   * @param cannyMinThreshold canny edge detection minimum threshold for the canny step
   * @param cannyMaxThreshold canny edge detection maximum threshold for the canny step
   */
  public DistanceFromRingTracker(
    double pctThreshold,
    int thresholdThresh,
    int thresholdMaxVal,
    int cannyMinThreshold,
    int cannyMaxThreshold
  ) {
    this.pctBoundsThreshold = pctThreshold;
    this.thresholdThresh = thresholdThresh;
    this.thresholdMaxVal = thresholdMaxVal;
    this.cannyMinThreshold = cannyMinThreshold;
    this.cannyMaxThreshold = cannyMaxThreshold;
  }

  @Override
  public Mat processFrame(Mat input) {
    contours.clear();
    Imgproc.cvtColor(input, gray, Imgproc.COLOR_RGB2GRAY);
    Imgproc.blur(gray, gray, new Size(3, 3));
    Imgproc.threshold(
      gray,
      threshold,
      thresholdThresh,
      thresholdMaxVal,
      Imgproc.THRESH_BINARY_INV + Imgproc.THRESH_OTSU
    );
    Imgproc.Canny(threshold, edges, cannyMinThreshold, cannyMaxThreshold);
    Imgproc.findContours(
      edges,
      contours,
      new Mat(),
      Imgproc.RETR_LIST,
      Imgproc.CHAIN_APPROX_SIMPLE
    );
    input.copyTo(output);
    //Imgproc.drawContours(output, contours, -1, new Scalar(225, 0, 0));
    MatOfPoint largestContour = max();

    if (largestContour != null) {
      largestContour.convertTo(largestContour2f, CvType.CV_32F);
      RotatedRect newBounds = Imgproc.minAreaRect(largestContour2f);
      double sizeDifference = newBounds.size.area() / bounds.size.area();
      if (
        sizeDifference > pctBoundsThreshold ||
        sizeDifference < -pctBoundsThreshold
      ) {
        bounds = newBounds;
      }
      Imgproc.rectangle(
        output,
        newBounds.boundingRect(),
        new Scalar(0, 255, 0)
      );
      Imgproc.rectangle(output, bounds.boundingRect(), new Scalar(255, 0, 0));
    }
    return output;
  }

  /**
   * Finds and returns the largest point in the contours.
   * Preforms a linear search
   *
   * @return MatofPoint the largest point in the contours
   * @see MatOfPoint
   */
  private MatOfPoint max() {
    if (contours.size() > 0) {
      MatOfPoint largest = contours.get(0);
      for (MatOfPoint point : contours.subList(1, contours.size())) {
        if (point.size().area() > largest.size().area()) {
          largest = point;
        }
      }
      return largest;
    }
    return null;
  }

  /**
   * Returns the distance from the detected ring in inches
   * Formula used is D’ = (W x F) / P
   *
   * @return double the computed distance from the white ring on screen in inches
   */
  public double computefocalLength(){
    return (bounds.size.width * distance) / 5;
  }
  public double computeDistance() {
    return (5 * focalLength) / bounds.size.width;
  }
}

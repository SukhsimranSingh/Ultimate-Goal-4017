package org.firstinspires.ftc.teamcode.ultimategoal.vision;

import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvTracker;

import java.util.ArrayList;

public class OpenCVTrackerApiPipeline extends OpenCvPipeline
{
    private ArrayList<OpenCvTracker> trackers = new ArrayList<>();
    private int trackerDisplayIdx = 0;

    public synchronized void addTracker(OpenCvTracker tracker)
    {
        trackers.add(tracker);
    }

    public synchronized void removeTracker(OpenCvTracker tracker)
    {
        trackers.remove(tracker);

        if(trackerDisplayIdx >= trackers.size())
        {
            trackerDisplayIdx--;

            if(trackerDisplayIdx < 0)
            {
                trackerDisplayIdx = 0;
            }
        }
    }

    @Override
    public synchronized Mat processFrame(Mat input)
    {
        if(trackers.size() == 0)
        {
            return input;
        }

        ArrayList<Mat> returnMats = new ArrayList<>(trackers.size());

        for(OpenCvTracker tracker : trackers)
        {
//            returnMats.add(tracker.processFrameInternal(input));
            returnMats.add(tracker.processFrame(input));
        }

        return returnMats.get(trackerDisplayIdx);
    }

    @Override
    public synchronized void onViewportTapped()
    {
        trackerDisplayIdx++;

        if(trackerDisplayIdx >= trackers.size())
        {
            trackerDisplayIdx = 0;
        }
    }
}
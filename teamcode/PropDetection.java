/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

class PropDetection extends OpenCvPipeline
{
    private Mat scaledImage = new Mat();
    private Mat blurredImage = new Mat();
    private Mat hslImage = new Mat();
    private Mat thresholded = new Mat();
    private ArrayList<MatOfPoint> contours = new ArrayList<MatOfPoint>();


    @Override
    public Mat processFrame(Mat input)
    {
        // Resize the image
        Imgproc.resize(input, scaledImage, new Size(0,0), CSConstants.imageScalingFactor, CSConstants.imageScalingFactor, Imgproc.INTER_LINEAR);

        Imgproc.blur(scaledImage, blurredImage, new Size(CSConstants.imageBlurKernelSize, CSConstants.imageBlurKernelSize));

        Imgproc.cvtColor(blurredImage, thresholded, Imgproc.COLOR_BGR2HLS);

        Core.inRange(thresholded, new Scalar(CSConstants.hueRange[0], CSConstants.luminanceRange[0], CSConstants.saturationRange[0]),
                                  new Scalar(CSConstants.hueRange[1], CSConstants.luminanceRange[1], CSConstants.saturationRange[1]), thresholded);

        Imgproc.findContours(thresholded, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

        System.out.println("Found: " + contours.size() + " contours");

        return thresholded;
    }

    public ArrayList<AprilTagDetection> getDetectionsUpdate()
    {
        synchronized (detectionsUpdateSync)
        {
            ArrayList<AprilTagDetection> ret = detectionsUpdate;
            detectionsUpdate = null;
            return ret;
        }
    }

}
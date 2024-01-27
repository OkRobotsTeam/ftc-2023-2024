package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;

import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicReference;

public class CSPropVisionProcessor implements VisionProcessor {
    private Mat scaledImage = new Mat();
    private Mat blurredImage = new Mat();
    private Mat hslImage = new Mat();
    private Mat thresholded = new Mat();

    private final AtomicReference<ArrayList<MatOfPoint>> contours = new AtomicReference<>();

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        // Code executed on the first frame dispatched into this VisionProcessor
    }
    
    @Override
    public Mat processFrame(Mat frame, long captureTimeNanos) {
        // Actual computer vision magic will happen here
        Imgproc.resize(frame, scaledImage, new Size(0,0), CSConstants.imageScalingFactor, CSConstants.imageScalingFactor, Imgproc.INTER_LINEAR);

        Imgproc.blur(scaledImage, blurredImage, new Size(CSConstants.imageBlurKernelSize, CSConstants.imageBlurKernelSize));


//        Mat leftCropZone = blurredImage.submat();

        Imgproc.cvtColor(blurredImage, thresholded, Imgproc.COLOR_BGR2HLS);

        Core.inRange(thresholded, new Scalar(CSConstants.hueRange[0], CSConstants.luminanceRange[0], CSConstants.saturationRange[0]),
                new Scalar(CSConstants.hueRange[1], CSConstants.luminanceRange[1], CSConstants.saturationRange[1]), thresholded);

//        A temporary array to store the contours before they are sent to the AtomicReference
        ArrayList<MatOfPoint> temp_contours = new ArrayList<>() ;

        Imgproc.findContours(thresholded, temp_contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

        contours.set(temp_contours);

        return thresholded;
    }
    
    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        Paint paint = new Paint();
        paint.setColor(Color.RED);
        paint.setStyle(Paint.Style.STROKE);
        paint.setStrokeWidth(4);
//
//        canvas.drawLine(100,100, 300,300, paint);
        Mat mat = (Mat) userContext;

        Bitmap bitmap = Bitmap.createBitmap(onscreenWidth, onscreenHeight, Bitmap.Config.ARGB_8888);
        Utils.matToBitmap(mat,bitmap);
        canvas.drawBitmap(bitmap,0,0,paint);


        // Cool feature: This method is used for drawing annotations onto
        // the displayed image, e.g outlining and indicating which objects
        // are being detected on the screen, using a GPU and high quality 
        // graphics Canvas which allow for crisp quality shapes.
    }
    
}
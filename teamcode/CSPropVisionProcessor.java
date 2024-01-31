package org.firstinspires.ftc.teamcode;

import static org.opencv.core.Core.countNonZero;
import static org.opencv.core.CvType.CV_8UC3;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Typeface;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

public class CSPropVisionProcessor implements VisionProcessor {
    volatile int detectedPropZone =0;
    private Alliance alliance = Alliance.BLUE;

    enum Alliance  {BLUE, RED};
    int oneThirdW = 213;
    int height = 320;
    int twoThirdsW = 417;
    int width = 640;
    Mat red;
    Mat teal;
    Mat blue;
    Mat white;
    Mat mask = new Mat();
    Mat tempMask = new Mat();
    Mat hsvFrame = new Mat();

    //x,y coordinates of the corners of the 3 zones.
    Point left1;
    Point left2;
    Point center1;
    Point center2;
    Point right1;
    Point right2;

    public void setAlliance(Alliance allianceIn) {
        alliance = allianceIn;
    }
    public Alliance getAlliance() {
        return alliance;
//        if (alliance == Alliance.RED) {
//            return "RED";
//        } else {
//            return "BLUE";
//        }
    }
    public int getDetectedPropZone() {
        return detectedPropZone;
    }

    public int getWidth() { return width;  }
    public int getHeight(){ return height; }

    private Paint boxPaint;
    private Paint textPaint;

    @Override
    public void init(int widthIn, int heightIn, CameraCalibration calibration) {
        width = widthIn/2;
        height = heightIn/2;
        oneThirdW = width/3;
        twoThirdsW = oneThirdW *2;

        left1 = new Point(0,0);
        left2 = new Point(oneThirdW,height);
        center1 = new Point(oneThirdW +1,0);
        center2 = new Point(twoThirdsW,height);
        right1 = new Point(twoThirdsW,0);
        right2 = new Point(width,height);

        red = new Mat(new Size(width,height), CV_8UC3, new Scalar(0, 0, 255));
        teal = new Mat(new Size(width,height), CV_8UC3, new Scalar(255, 255, 0));
        blue = new Mat(new Size(width,height), CV_8UC3, new Scalar(255, 0, 0));
        white = new Mat(new Size(width,height), CV_8UC3, new Scalar(255, 255, 255));

        textPaint = new Paint();
        textPaint.setColor(Color.WHITE);
        textPaint.setTypeface(Typeface.DEFAULT_BOLD);
        textPaint.setTextSize(30);
        textPaint.setAntiAlias(true);

        boxPaint = new Paint();
        boxPaint.setColor(Color.BLACK);
        boxPaint.setStyle(Paint.Style.FILL);

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Imgproc.resize(frame, frame, new Size(width,height), 0, 0, Imgproc.INTER_LINEAR);
        Imgproc.cvtColor(frame, frame, Imgproc.COLOR_RGB2BGR);
        Imgproc.blur(frame, frame, new Size(CSConstants.imageBlurKernelSize, CSConstants.imageBlurKernelSize));
        Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_BGR2HSV);
        if (alliance == Alliance.RED) {
            Core.inRange(hsvFrame, new Scalar(170, 70, 50), new Scalar(180, 255, 255), mask);
        } else {
            Core.inRange(hsvFrame, new Scalar(100, 70, 50), new Scalar(130, 255, 255), mask);
        }
        red.copyTo(frame,mask);


//        Core.inRange(frame, new Scalar(0 ,0, 128 ),
//                new Scalar(100, 80, 255), mask);
//        Core.inRange(frame, new Scalar(0 ,0, 128 ),
//                new Scalar(100, 80, 255), mask);
//
//        pink.copyTo(frame,mask);
//
//        Core.inRange(frame, new Scalar(0 ,0, 70 ),
//                new Scalar(60, 50, 255), tempMask);
//
//        teal.copyTo(frame,tempMask);
//        tempMask.copyTo(mask,tempMask);
//
//
//        Core.inRange(frame, new Scalar(0 ,0, 220 ),
//                new Scalar(180, 160, 255), tempMask);
//        blue.copyTo(frame,tempMask);
//        tempMask.copyTo(mask,tempMask);
//
//        //redImg.copyTo(frame,mask);



        Mat leftMat  = mask.submat(new Rect(left1,left2));
        Mat centerMat  = mask.submat(new Rect(center1,center2));
        Mat rightMat  = mask.submat(new Rect(right1,right2));
        int leftCount = countNonZero(leftMat);
        int centerCount = countNonZero(centerMat);
        int rightCount = countNonZero(rightMat);
        detectedPropZone = 0;
        if (leftCount > centerCount) {
            if (leftCount > rightCount) {
                detectedPropZone = 1;
            } else {
                detectedPropZone = 3;
            }
        } else {
            if (centerCount > rightCount) {
                detectedPropZone = 2;
            } else {
                detectedPropZone = 3;
            }
        }
        Scalar blue = new Scalar(255,0,0);

        if (detectedPropZone == 1) {
            Imgproc.rectangle(frame, left1, left2, blue, 2);
        } else if (detectedPropZone == 2) {
            Imgproc.rectangle(frame, center1,center2, blue, 2);
        } else if (detectedPropZone == 3) {
            Imgproc.rectangle(frame, right1, right2, blue, 2);
        } else {
            Imgproc.rectangle(frame, left1, right2, blue, 2);
        }
        Imgproc.cvtColor(frame, frame, Imgproc.COLOR_BGR2RGB);
        return mask; // No context object    }

    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        canvas.drawRect(new android.graphics.Rect(0, 0, 385, 45), boxPaint);
        canvas.drawText("Zone Detected "+ detectedPropZone, 5, 33, textPaint);
    }

}
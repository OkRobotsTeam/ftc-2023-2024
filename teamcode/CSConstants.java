package org.firstinspires.ftc.teamcode;


public class CSConstants {
    public final static double imageScalingFactor = 0.25;
    public final static double imageBlurKernelSize = 4;
    public final static int imageWidth = 640;
    public final static int imageHeight = 480;
    public final static double[] hueRange = {0, 100};
    public final static double[] luminanceRange = {0, 100};
    public final static double[] saturationRange = {0, 100};

    public final static double arm_encoder_counts_per_revolution = 3895.9;

    public final static int wristReadyForPickup = 0;

    // Dock/Undock values for shoulder, elbow, and wrist
    public final static int shoulderReadyForDockUndock = 200;
    public final static int elbowReadyForDockUndock = 200;
    public final static int wristUpright = 1;
    public final static double wristDocking = 0;
    public final static long wristMoveMilliseconds = 200;

    public final static int elbowTolerance = 10;
    public final static int shoulderTolerance = 10;
    public final static double elbowPower = 0.5;
    public final static double shoulderPower = 0.5;

//    public final static Pair<Double, Double> defaultFreePosition = new Pair<Double, Double>(0.4, 0.3);
    public final static int shoulderDefaultFreePosition = 100;
    public final static int elbowDefaultFreePosition = 100;

    public final static int elbowAdjustmentSize = 100;

    public final static int shoulderDocked = 0;
    public final static int elbowDocked = 0;

    public final static double leftFingerOpenPosition = 0;
    public final static double rightFingerOpenPosition = 0;
    public final static double leftFingerClosedPosition = 0;
    public final static double rightFingerClosedPosition = 0;


}

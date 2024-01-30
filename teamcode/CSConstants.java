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

    // Dock/Undock values for shoulder, elbow, and wrist
    public final static int shoulderReadyForDockUndock = 200;
    public final static int elbowReadyForDockUndock = 150;
    public final static double wristDocking = 0.5;
    public final static double wristPickup = 0.9;
    public final static long wristMoveMilliseconds = 500;

    public final static int elbowTolerance = 10;
    public final static int shoulderTolerance = 10;
    public final static double elbowPower = 0.5;
    public final static double shoulderPower = 0.5;

    public final static int shoulderDefaultFreePosition = 400;
    public final static int elbowDefaultFreePosition = 400;

    public final static int elbowAdjustmentSize = 100;

    public final static int[][] armPositions = {
            // 0,    1,    2,    3,   4,   5,   6,    7,    8,    9,   10
            {  0,  1200,  950,  883, 883, 883, 952,  1044,  1209, 1000, 1100},   //shoulder
            {  0,  326,  409,  558, 634, 798, 1048, 1261, 1594, 1500, 1700} }; //elbow
    public final static double[] wristPositions =
            //  0,    1,    2,    3,   4,   5,   6,   7,   8,   9,  10
            {0.35, 0.35, 0.40, 0.45,0.50,0.55,0.65,0.70,0.75,0.75,0.75};
    public final static int shoulderDocked = 0;
    public final static int elbowDocked = 0;

    public final static double leftFingerOpenPosition = 1;
    public final static double rightFingerOpenPosition = 0.5;
    public final static double leftFingerClosedPosition = 0;
    public final static double rightFingerClosedPosition = 1;

    public final static double endgameRuntimeSeconds = 1;

    public final static double flipperPower = 1;

    public final static double beltPower = 1;



}

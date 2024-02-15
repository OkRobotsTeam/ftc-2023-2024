package org.firstinspires.ftc.teamcode;


public class CSConstants {
    public final static double imageScalingFactor = 0.25;
    public final static double imageBlurKernelSize = 9;
    public final static int imageWidth = 640;
    public final static int imageHeight = 480;
    public final static double[] hueRange = {0, 100};
    public final static double[] luminanceRange = {0, 100};
    public final static double[] saturationRange = {0, 100};

    public final static double arm_encoder_counts_per_revolution = 3895.9;

    // Dock/Undock values for shoulder, elbow, and wrist
    public final static int shoulderDocked = 0;
    public final static int elbowDocked = 10;

    public final static int shoulderReadyForDockUndock = 0;
    public final static int elbowReadyForDockUndock = 200;
    public final static double wristDocking = 0.5;
    public final static double wristPickup = 0.85;
    public final static long wristMoveMilliseconds = 500;

    public final static int elbowTolerance = 10;
    public final static int shoulderTolerance = 10;
    public final static double elbowPower = 0.5;
    public final static double shoulderPower = 0.5;

    public final static int shoulderDefaultFreePosition = 400;

    public final static int elbowAdjustmentSize = 100;

    public static int numArmPositions = 4;

    public final static int[][] armPositions = {
            // 0,    1,    2,    3,   4
            {  0,    0,  990,  969, 1130}, //shoulder
            {  0,  200,  671,  1170, 1651} }; //elbow
    public final static double[] wristPositions =
            //  0,    1,    2,    3,   4
            {0.35, 0.50, 0.41, 0.56,0.68};

    public final static int elbowDefaultFreePosition = armPositions[1][1];

    public final static double leftFingerOpenPosition = 1;
    public final static double rightFingerOpenPosition = 0.5;
    public final static double leftFingerClosedPosition = 0;
    public final static double rightFingerClosedPosition = 1;

    public final static double endgameRuntimeSeconds = 1;

    public final static double flipperPower = 0.8;

    public final static double beltPower = 1;
    public static double elbowPowerLow = 0.3;
}

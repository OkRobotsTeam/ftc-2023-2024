package org.firstinspires.ftc.teamcode;


import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class CSConstants {
    public static class Sensors {
        public final static String IMU = "imu";
        // Axes order for the IMU, the first axis will be used (Z in the case of AxesOrder.ZYX)
        public final static AxesOrder IMUAxesOrder = AxesOrder.ZYX;
        public final static AxesReference IMUAxesReference = AxesReference.INTRINSIC;

    }

    public static class Drivetrain {
        public final static String frontLeft = "left_front";
        public final static String backLeft = "left_back";
        public final static String frontRight = "right_front";
        public final static String backRight = "right_back";

        public final static double wheelDiameterIn = 4.0;
        public final static double wheelCircumferenceIn = wheelDiameterIn * Math.PI;
        public final static double encoderCountsPerRevolution = 28;  // HD Hex motor encoders have a resolution of 28 counts per revolution at the motor
        public final static double motorGearRatio = 20.0;  // A gear ratio of 20:1 (input:output)
        public final static double encoderOutputCountsPerRevolution = encoderCountsPerRevolution * motorGearRatio;
        public final static double encoderCountsPerIn = encoderOutputCountsPerRevolution / wheelCircumferenceIn;


        // PID parameters
        public final static double drivetrainTurningPIDKp = 1;
        public final static double drivetrainTurningPIDKi = 0;
        public final static double drivetrainTurningPIDKd = 0;
        public final static double drivetrainTurningPIDIntegralLimit = 1;
        public final static double drivetrainTurningPIDTimeStep = 0.05;

        public final static boolean isFieldCentricDuringTeleOp = false;
        public final static double teleOpMinimumMovementPower = 0.5;
        public final static double teleOpMaximumMovementPower = 1;

        public final static double teleOpMinimumTurnPower = 0.5;
        public final static double teleOpMaximumTurnPower = 1;
    }


    public final static String IMUCalibrationFilename = "AdafruitIMUCalibration.json";

    public final static double imageScalingFactor = 0.25;
    public final static double imageBlurKernelSize = 9;
    public final static int imageWidth = 640;
    public final static int imageHeight = 480;
    public final static double[] hueRange = {0, 100};
    public final static double[] luminanceRange = {0, 100};
    public final static double[] saturationRange = {0, 100};

    public final static double arm_encoder_counts_per_revolution = 3895.9;

    // Dock/Undock values for shoulder, elbow, and wrist
    public final static int shoulderReadyForDockUndock = 0;
    public final static int elbowReadyForDockUndock = 150;
    public final static double wristDocking = 0.5;
    public final static double wristPickup = 0.95;
    public final static long wristMoveMilliseconds = 500;

    public final static int elbowTolerance = 10;
    public final static int shoulderTolerance = 10;
    public final static double elbowPower = 0.5;
    public final static double shoulderPower = 0.5;

    public final static int shoulderDefaultFreePosition = 400;
    public final static int elbowDefaultFreePosition = 400;

    public final static int elbowAdjustmentSize = 100;

    public static int numArmPositions = 4;

    public final static int[][] armPositions = {
            // 0,    1,    2,    3,   4
            {0, 0, 990, 969, 1130}, //shoulder
            {0, 370, 671, 1170, 1651}}; //elbow
    public final static double[] wristPositions =
            //  0,    1,    2,    3,   4
            {0.35, 0.50, 0.41, 0.56, 0.68};
    public final static int shoulderDocked = 0;
    public final static int elbowDocked = 0;

    public final static double leftFingerOpenPosition = 1;
    public final static double rightFingerOpenPosition = 0.5;
    public final static double leftFingerClosedPosition = 0;
    public final static double rightFingerClosedPosition = 1;

    public final static double endgameRuntimeSeconds = 1;

    public final static double flipperPower = 0.5;

    public final static double beltPower = 1;


}

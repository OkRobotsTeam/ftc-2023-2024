package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.ObjectInputStream;

/**
 * A class representing a re-written mecanum drivetrain controller
 */
public class BetterMecanumDrive {

    // Motors
    private final DcMotorEx[] motors = new DcMotorEx[4];

    // IMU
    BNO055IMU imu;
    Orientation initialOrientation;

    // PID controller for rotation
    PIDController rotationPID = new PIDController(1, 0, 0, 0.05, 1);

    // Constants for motor positions
    private final int FRONT_LEFT = 0;
    private final int FRONT_RIGHT = 2;
    private final int BACK_LEFT = 1;
    private final int BACK_RIGHT = 3;
    private final int[] WHEEL_POSITIONS = {FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT};

    // State Variables
    private double[] encoderOffsets = {0.0, 0.0, 0.0, 0.0};

    /**
     * Initialize the Mecanum drive with specified parameters.
     *
     * @param hardwareMap Hardware map from the OpMode.
     */
    BetterMecanumDrive(HardwareMap hardwareMap) {
        // Initialize motors
        motors[FRONT_LEFT] = hardwareMap.get(DcMotorEx.class, CSConstants.Drivetrain.frontLeft);
        motors[BACK_LEFT] = hardwareMap.get(DcMotorEx.class, CSConstants.Drivetrain.backLeft);
        motors[FRONT_RIGHT] = hardwareMap.get(DcMotorEx.class, CSConstants.Drivetrain.frontRight);
        motors[BACK_RIGHT] = hardwareMap.get(DcMotorEx.class, CSConstants.Drivetrain.backRight);
        setupIMU(hardwareMap);
    }

    /**
     * Setup IMU with specified parameters.
     *
     * @param hardwareMap Hardware map from the OpMode.
     */
    private void setupIMU(HardwareMap hardwareMap) {
        // Create the IMU parameters object
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        // Configure the IMU parameters
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        // Initialize the IMU using the hardware map
        imu = hardwareMap.get(BNO055IMU.class, CSConstants.Sensors.IMU);
        imu.initialize(parameters);

        // Load the IMU calibration data
        loadIMUCalibration(CSConstants.IMUCalibrationFilename);

        // Get the initial orientation from IMU
        initialOrientation = getOrientation();
    }


    /**
     * Load IMU calibration data from file.
     *
     * @param filename Name of the calibration file.
     */
    public void loadIMUCalibration(String filename) {
        BNO055IMU.CalibrationData calibrationData;

        try {
            // Load calibration data from the specified file
            File file = AppUtil.getInstance().getSettingsFile(filename);
            FileInputStream fileInputStream = new FileInputStream(file);
            ObjectInputStream objectInputStream = new ObjectInputStream(fileInputStream);

            // Read the calibration data object from the input stream
            calibrationData = (BNO055IMU.CalibrationData) objectInputStream.readObject();

            // Close the input streams
            fileInputStream.close();
            objectInputStream.close();

            // Write the loaded calibration data to the IMU
            imu.writeCalibrationData(calibrationData);
        } catch (IOException exception) {
            // Handle IO exceptions (e.g., file not found, read/write error)
            System.out.println("loadIMUCalibration: IOException: " + exception.getMessage());
        } catch (ClassNotFoundException exception) {
            // Handle class not found exceptions (e.g., when deserializing the object)
            System.out.println("loadIMUCalibration: ClassNotFoundException: " + exception.getMessage());
        } catch (Exception exception) {
            // Handle other exceptions that may occur during the process
            System.out.println("loadIMUCalibration: Exception: " + exception.getMessage());
        }
    }


    /**
     * Get current orientation from IMU.
     *
     * @return Current orientation.
     */
    Orientation getOrientation() {
        return imu.getAngularOrientation(CSConstants.Sensors.IMUAxesReference, CSConstants.Sensors.IMUAxesOrder, AngleUnit.DEGREES);
    }

    /**
     * Get the current heading (rotation around the Z-axis) from the IMU.
     *
     * @return Current heading in degrees.
     */
    public double getHeading() {
        return getOrientation().firstAngle;
    }

    /**
     * Get wheel position in inches for a specific wheel.
     *
     * @param wheel Index of the wheel.
     * @return Wheel position in inches.
     */
    private double getWheelPositionIn(int wheel) {
        return (motors[wheel].getCurrentPosition() - encoderOffsets[wheel]) * CSConstants.Drivetrain.encoderCountsPerIn;
    }

    /**
     * Set power for a specific wheel.
     *
     * @param wheel Index of the wheel.
     * @param power Power to set for the wheel (-1 to 1).
     */
    private void setWheelPower(int wheel, double power) {
        motors[wheel].setPower(power);
    }

    /**
     * Set powers for all wheels.
     *
     * @param powers Array of powers for each wheel (-1 to 1).
     */
    private void setWheelPowers(double[] powers) {
        // Iterate over each wheel position in the array
        for (int wheelPosition : WHEEL_POSITIONS) {
            // Set the power for the current wheel to the corresponding value in the powers array
            setWheelPower(wheelPosition, powers[wheelPosition]);
        }
    }

    /**
     * Set the current position for a specific wheel by updating its encoder offset.
     *
     * @param wheel     Index of the wheel whose position is to be set.
     * @param position  New current position (in encoder counts) for the specified wheel.
     *                 This method updates the encoder offset for the wheel, which is used
     *                 to track the distance traveled by the robot.
     */
    private void setWheelPosition(int wheel, double position) {
        // Calculate the encoder offset for the specified wheel by subtracting
        // the desired position from the current position of the wheel
        encoderOffsets[wheel] = getWheelPositionIn(wheel) - position;
    }


    /**
     * Calculate the total distance traveled by the robot.
     *
     * @return Total distance traveled in inches.
     */
    public double getDistanceTraveledIn() {
        // Calculate the average distance traveled by the left wheels
        double leftDistanceIn = (getWheelPositionIn(FRONT_LEFT) + getWheelPositionIn(BACK_LEFT)) / 2;

        // Calculate the average distance traveled by the right wheels
        double rightDistanceIn = (getWheelPositionIn(FRONT_RIGHT) + getWheelPositionIn(BACK_RIGHT)) / 2;

        // Calculate and return the average distance traveled by both sides
        return (leftDistanceIn + rightDistanceIn) / 2;
    }

    /**
     * Reset the distance traveled by resetting encoder offsets.
     */
    public void resetDistanceTravelled() {
        // Iterate over all motors to reset their encoder offsets
        for (int motor : WHEEL_POSITIONS) {
            // Set the encoder offset for the current motor to zero,
            // effectively resetting the distance traveled for that motor
            setWheelPosition(motor, 0);
        }
    }

    /**
     * Scale wheel speeds if any of them exceeds the maximum allowed speed.
     *
     * @param wheelSpeeds Array of wheel speeds (-1 to 1).
     * @return Scaled wheel speeds.
     */
    double[] desaturateWheelSpeeds(double[] wheelSpeeds) {
        // Initialize a variable to store the maximum absolute speed
        double max = 0.0;

        // Find the maximum absolute value in the array
        for (double speed : wheelSpeeds) {
            max = Math.max(max, Math.abs(speed));
        }

        // If the maximum speed exceeds 1, scale all wheel speeds down to avoid clipping and loss of control
        if (max > 1) {
            // Scale each wheel speed proportionally to ensure the maximum speed is 1
            for (int wheelPosition : WHEEL_POSITIONS) {
                wheelSpeeds[wheelPosition] /= max; // Scale the speed of the wheel
            }
        }

        // Return the scaled wheel speeds
        return wheelSpeeds;
    }


    /**
     * Move the robot forward with IMU feedback.
     *
     * @param distanceIn Distance to move in inches.
     * @param speed      Speed at which to move (-1 to 1).
     */
    public void moveForwardWithIMU(double distanceIn, double speed) {
        // Determine the initial rotation angle for maintaining direction
        double targetRotation = getOrientation().firstAngle;

        // Reset distance traveled to start tracking from the current position
        resetDistanceTravelled();

        // Continue moving until the specified distance is reached
        while (getDistanceTraveledIn() < distanceIn) {
            // Calculate the correction to maintain the initial direction
            double rotationalPIDOutput = rotationPID.update(getOrientation().firstAngle - targetRotation);

            // Set initial wheel powers (all wheels moving forward)
            double[] wheelPowers = {speed, speed, speed, speed};

            // Apply correction to left wheels (front and back)
            wheelPowers[FRONT_LEFT] += rotationalPIDOutput;
            wheelPowers[BACK_LEFT] += rotationalPIDOutput;

            // Apply correction to right wheels (front and back)
            wheelPowers[FRONT_RIGHT] -= rotationalPIDOutput;
            wheelPowers[BACK_RIGHT] -= rotationalPIDOutput;

            // Desaturate wheel speeds to keep them within the valid range (-1 to 1)
            wheelPowers = desaturateWheelSpeeds(wheelPowers);

            // Update the wheels with the calculated powers
            setWheelPowers(wheelPowers);
        }
    }

    /**
     * Move the robot at a specified angle with a given speed.
     *
     * @param angle        Angle at which to move (in degrees with positive being clockwise).
     * @param speed        Speed at which to move (-1 to 1).
     * @param fieldCentric Flag indicating whether the angle is field-centric or robot-centric
     *                     If true, the angle is interpreted as a field-centric angle; if false,
     *                     it's interpreted as a robot-centric angle.
     */
    public void move(double angle, double speed, boolean fieldCentric) {
        double[] motorSpeeds = {speed, speed, speed, speed};
        // TODO: Implement move functionality here
    }
}
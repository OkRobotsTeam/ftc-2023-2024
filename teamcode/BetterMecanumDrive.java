package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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

    // Telemetry
    private Telemetry telemetry;

    // Debugging
    private boolean pauseBeforeEachMovement = false;
    private LinearOpMode opMode;


    // Constants for wheel positions
    private final int FRONT_LEFT = 0;
    private final int FRONT_RIGHT = 2;
    private final int BACK_LEFT = 1;
    private final int BACK_RIGHT = 3;

    // An array of all the wheel positions for iteration
    private final int[] WHEEL_POSITIONS = {FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT};

    // The directions in which each wheel applies force when spinning forward, in radians in counterclockwise-positive notation
    private final double[] WHEEL_FORCE_DIRECTIONS_RAD = {0.78539816339, 2.35619449019, 2.35619449019, 0.78539816339};

    // State Variables
    private double[] encoderOffsets = {0.0, 0.0, 0.0, 0.0};

    private double[] odometryLastMotorPositions = {0, 0, 0, 0};
 
    private double[] odometryPosition = {0.0d, 0.0d};

    /**
     * Initialize the Mecanum drive with specified parameters.
     *
     * @param hardwareMap Hardware map from the OpMode.
     */
    public BetterMecanumDrive(HardwareMap hardwareMap, Telemetry telemetry, LinearOpMode opMode) {
        // Initialize motors
        motors[FRONT_LEFT] = hardwareMap.get(DcMotorEx.class, CSConstants.Drivetrain.frontLeft);
        motors[BACK_LEFT] = hardwareMap.get(DcMotorEx.class, CSConstants.Drivetrain.backLeft);
        motors[FRONT_RIGHT] = hardwareMap.get(DcMotorEx.class, CSConstants.Drivetrain.frontRight);
        motors[BACK_RIGHT] = hardwareMap.get(DcMotorEx.class, CSConstants.Drivetrain.backRight);
        setupIMU(hardwareMap);
        this.telemetry = telemetry;
        this.opMode = opMode;
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
     * Set the directions of the motors for each wheel.
     *
     * @param directions Array of directions for each wheel to turn in order to go forward.
     *                   The array length should be equal to the number of wheels.
     */
    void setMotorDirections(DcMotorSimple.Direction[] directions) {
        // Iterate over each wheel position
        for (int wheelPosition : WHEEL_POSITIONS) {
            // Set the direction of the motor at the wheel position
            motors[wheelPosition].setDirection(directions[wheelPosition]);
        }
    }

    void enableDebugPause() {
        pauseBeforeEachMovement = true;
    }

    void disableDebugPause() {
        pauseBeforeEachMovement = false;
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
     * @param wheel    Index of the wheel whose position is to be set.
     * @param position New current position (in encoder counts) for the specified wheel.
     *                 This method updates the encoder offset for the wheel, which is used
     *                 to track the distance traveled by the robot.
     */
    private void setWheelPosition(int wheel, double position) {
        // Calculate the encoder offset for the specified wheel by subtracting
        // the desired position from the current position of the wheel
        encoderOffsets[wheel] = getWheelPositionIn(wheel) - position;
    }


    /**
     * Return the distance of the robot from a point
     *
     * @return Distance traveled in inches.
     */
    public double getCurrentDistanceFromPoint(double[] point) {
        double[] currentPosition = getOdometryPosition();
        return Math.hypot(point[0] - currentPosition[0], point[1] - currentPosition[1]);
    }

    public void updateOdometry() {
        double currentFrontLeftPosition = getWheelPositionIn(FRONT_LEFT);
        double currentFrontRightPosition = getWheelPositionIn(FRONT_RIGHT);
        double currentBackLeftPosition = getWheelPositionIn(BACK_LEFT);
        double currentBackRightPosition = getWheelPositionIn(BACK_RIGHT);

        double frontLeftDistanceThisTick = currentFrontLeftPosition - odometryLastMotorPositions[FRONT_LEFT];
        double frontRightDistanceThisTick = currentFrontRightPosition - odometryLastMotorPositions[FRONT_LEFT];
        double backLeftDistanceThisTick = currentBackLeftPosition - odometryLastMotorPositions[BACK_LEFT];
        double backRightDistanceThisTick = currentBackRightPosition - odometryLastMotorPositions[BACK_RIGHT];

        odometryLastMotorPositions[FRONT_LEFT] = currentFrontLeftPosition;
        odometryLastMotorPositions[FRONT_RIGHT] = currentFrontRightPosition;
        odometryLastMotorPositions[BACK_LEFT] = currentBackLeftPosition;
        odometryLastMotorPositions[BACK_RIGHT] = currentBackRightPosition;

        double deltaY = (frontLeftDistanceThisTick + frontRightDistanceThisTick + backLeftDistanceThisTick + backRightDistanceThisTick) / 4;
        double deltaX = (frontLeftDistanceThisTick - frontRightDistanceThisTick - backLeftDistanceThisTick + backRightDistanceThisTick) / 4;

        odometryPosition[0] += deltaX * Math.cos(Math.toRadians(getHeading()));
        odometryPosition[1] += deltaY * Math.sin(Math.toRadians(getHeading()));
    }

    public double[] getOdometryPosition() {
        return odometryPosition;
    }

    public void setOdometryPosition(double[] position) {
        odometryPosition = position;
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
     * @param wheelSpeeds Array of wheel speeds (approximately -1 to 1).
     * @return Scaled wheel speeds.
     */
    double[] desaturateWheelPowers(double[] wheelSpeeds) {
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
     * Move the robot towards a target direction while maintaining a target heading with IMU feedback.
     *
     * @param distanceIn Distance to move in inches.
     * @param power      Speed at which to move (-1 to 1).
     */
    public void moveWithIMU(double targetRotation, double moveAngle, double distanceIn, double power) {
        if (pauseBeforeEachMovement) {
            while (!opMode.gamepad1.a && !opMode.isStopRequested()) {}
        }

        // Reset distance traveled to start tracking from the current position
        resetDistanceTravelled();

        setOdometryPosition(new double[] {0, 0});

        double[] initialPosition = getOdometryPosition();

        // Continue moving until the specified distance is reached
        while (getCurrentDistanceFromPoint(initialPosition) < distanceIn) {
            // Calculate the correction to maintain the target direction
            double rotationalPIDOutput = rotationPID.update(getOrientation().firstAngle - targetRotation);

            move(0, power, rotationalPIDOutput, false);
        }
    }

    /**
     * Turn to an angle with IMU feedback.
     *
     * @param angle     Angle to turn to in degrees.
     * @param power     Speed at which to turn (-1 to 1).
     */
    public void turnToWithIMU(double angle, double power, double angleTolerance) {
        if (pauseBeforeEachMovement) {
            while (!opMode.gamepad1.a && !opMode.isStopRequested()) {}
        }

        // Determine the target angle to turn to
        double targetRotation = angle;

        // Reset distance traveled to start tracking from the current position
        resetDistanceTravelled();

        // Update the PID here so that the previous_error variable is updated
        rotationPID.update(getOrientation().firstAngle - targetRotation);

        // Continue moving until the specified distance is reached
        while (rotationPID.previous_error > angleTolerance) {
            // Calculate the correction to maintain the initial direction
            double rotationalPIDOutput = rotationPID.update(getOrientation().firstAngle - targetRotation);

            // Set initial wheel powers (all wheels moving forward)
            double[] wheelPowers = {0, 0, 0, 0};

            // Apply correction to left wheels (front and back)
            wheelPowers[FRONT_LEFT] += rotationalPIDOutput;
            wheelPowers[BACK_LEFT] += rotationalPIDOutput;

            // Apply correction to right wheels (front and back)
            wheelPowers[FRONT_RIGHT] -= rotationalPIDOutput;
            wheelPowers[BACK_RIGHT] -= rotationalPIDOutput;

            // Desaturate wheel speeds to keep them within the valid range (-1 to 1)
            wheelPowers = desaturateWheelPowers(wheelPowers);

            // Update the wheels with the calculated powers
            setWheelPowers(wheelPowers);
        }
    }

    public void turnToWithIMU(double angle, double power) {
        turnToWithIMU(angle, power, 3);
    }

    /**
     * Calculate the necessary wheel power for a wheel pointing in the specified angle to move the robot toward the desired target
     * This function must be run for all wheels in the drivetrain separately
     *
     * @param movement_angle_rad The angle to move the robot
     * @param movement_speed     The speed to move at
     * @param wheel_angle_rad    The angle of the wheel to calculate power for
     * @return The calculated power for the wheel
     */

    double calculate_wheel_power(double movement_angle_rad, double movement_speed, double wheel_angle_rad) {
        if (movement_speed < 0) {
            throw new IllegalArgumentException("Speed may not be negative");
        }

        return movement_speed * Math.cos(wheel_angle_rad - movement_angle_rad);
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
    public void move(double angle, double speed, double spin, boolean fieldCentric) {
        // Array to store the powers for each wheel
        double[] wheelPowers = {0, 0, 0, 0};

        // If the movement is field-centric, adjust the angle based on the robot's current heading
        if (fieldCentric) {
            angle -= getHeading();  // Factor out our current heading from the target heading
        }

        // Calculate the wheel powers for each wheel based on the angle, speed, and wheel force directions
        wheelPowers[FRONT_LEFT] = calculate_wheel_power(angle, speed, WHEEL_FORCE_DIRECTIONS_RAD[FRONT_LEFT]) + spin;
        wheelPowers[BACK_LEFT] = calculate_wheel_power(angle, speed, WHEEL_FORCE_DIRECTIONS_RAD[BACK_LEFT]) + spin;
        wheelPowers[FRONT_RIGHT] = calculate_wheel_power(angle, speed, WHEEL_FORCE_DIRECTIONS_RAD[FRONT_RIGHT]) - spin;
        wheelPowers[BACK_RIGHT] = calculate_wheel_power(angle, speed, WHEEL_FORCE_DIRECTIONS_RAD[BACK_RIGHT]) - spin;

        wheelPowers = desaturateWheelPowers(wheelPowers);

        // Apply the calculated wheel powers to the motors
        setWheelPowers(wheelPowers);
        updateOdometry();
    }

    public void moveWithController(double leftStickX, double leftStickY, double rightStickX, double movementPowerMultiplier, double rotationPowerMultiplier, boolean fieldCentric) {
        double target_movement_angle = Math.atan2(leftStickY, leftStickX);
        double speed = Math.hypot(leftStickX, leftStickY) * movementPowerMultiplier;
        double spin = rightStickX * rotationPowerMultiplier;

        if (telemetry != null) {
            telemetry.addData("TargetMovementAngle", target_movement_angle);
            telemetry.addData("TargetSpeed", speed);
            telemetry.addData("TargetSpin", speed);
        }

        move(target_movement_angle, speed, spin, fieldCentric);
    }

    void telemetryDisplayMotorPosition() {
        if (telemetry == null) {return;}

        telemetry.addData("Motor Positions", "FRONT_LEFT:(%5.2f) FRONT_RIGHT:(%5.2f) BACK_LEFT:(%5.2f), BACK_RIGHT:(%5.2f)", getWheelPositionIn(FRONT_LEFT), getWheelPositionIn(FRONT_RIGHT), getWheelPositionIn(BACK_LEFT), getWheelPositionIn(BACK_RIGHT));
    }
    void telemetryDisplayMotorPower() {
        if (telemetry == null) {return;}

        telemetry.addData("Motor Velocities", "FRONT_LEFT:(%5.2f) FRONT_RIGHT:(%5.2f) BACK_LEFT:%(%5.2f) BACK_RIGHT:(%5.2f) ", motors[FRONT_LEFT].getPower(), motors[FRONT_RIGHT].getPower(), motors[BACK_LEFT].getPower(), motors[BACK_RIGHT].getPower());
    }

}
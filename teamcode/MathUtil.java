package org.firstinspires.ftc.teamcode;

public class MathUtil {
    public static double interpolate2D(double x1, double y1, double x2, double y2, double x) {
        // Ensure x1 is less than x2
        if (x1 > x2) {
            return interpolate2D(x2, y2, x1, y1, x);
        }

        // Calculate the slope (m) of the line passing through (x1, y1) and (x2, y2)
        double m = (y2 - y1) / (x2 - x1);

        // Calculate the interpolated value of y at point x using the linear equation: y = mx + b
        double y = m * (x - x1) + y1;

        return y;
    }

    public static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
    public static int clamp(int value, int min, int max) {
        return Math.max(min, Math.min(max, value));
    }
}

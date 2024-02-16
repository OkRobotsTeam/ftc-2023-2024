package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.MathUtil.clamp;

public class PIDController {
    // Tunings
    double kp, ki, kd, time_step, integral_limit;

    // State variables
    double previous_time;
    double current_value, target_value;
    double error_integral, error_derivative;
    double previous_error;
    double control_output;


    PIDController(double kp, double ki, double kd, double t, double integral_limit) {
        /*
        Initializes a PIDController instance.
        :param timer: The timer object used to measure time.
        p: Kp value for the PID.
        ki: Ki value for the PID.
        kd: Kd value for the PID.
        t: Minimum time between update calls. All calls made before this amount of time has passed since the last calculation will be ignored.
        integral_limit: The maximum absolute value for the integral term to prevent windup.
         */

        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.integral_limit = integral_limit;

        time_step = t;
        previous_time = System.currentTimeMillis() / 1000.0;
    }

    public double getKp() {
        return kp;
    }

    public double getKi() {
        return ki;
    }

    public double getKd() {
        return kd;
    }

    public void setKp(double kp) {
        this.kp = kp;
    }

    public void setKi(double ki) {
        this.ki = ki;
    }

    public void setKd(double kd) {
        this.kd = kd;
    }

    public double getSetpoint() {
        return target_value;
    }

    public void setSetpoint(double setpoint) {
        target_value = setpoint;
    }

    public void reset() {
        error_integral = 0;
        control_output = 0;
        previous_error = target_value - current_value;
    }

    public double update(double measurement) {
        double current_time = System.currentTimeMillis() / 1000.0;
        double delta_time = current_time - previous_time;

        if (delta_time < time_step) {
            return control_output;
        }

        double error = target_value - measurement;
        error_integral += error * delta_time;

        if (ki != 0) {
            error_integral = clamp(error_integral, -integral_limit, integral_limit);
        }

        error_derivative = (error - previous_error) / delta_time;
        control_output = kp * error + ki * error_integral + kd * error_derivative;
        previous_error = error;
        previous_time = System.currentTimeMillis() / 1000.0;

        return control_output;
    }
}

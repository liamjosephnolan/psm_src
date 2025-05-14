#include "config.h"
#include <Arduino.h>


void ramp(float current_angle, float target_angle, int axis_index, float ramp_time) {
    // Number of steps for the ramp
    const int steps = 100;

    // Calculate the step increment and time delay per step
    float step_increment = (target_angle - current_angle) / (steps - 1);
    float time_delay = (ramp_time / steps) * 1000.0f; // Convert seconds to milliseconds

    // Ramp through the steps
    for (int i = 0; i < steps; i++) {
        // Calculate the target position for this step
        float target_position = current_angle + i * step_increment;

        // Use the appropriate controller based on the axis index
        if (axis_index == 1) { // Pitch axis (LQI controller)
            float pitch_lqi_gains[3] = {220.6809f, 0.3174f, 0.1414f}; // Gains for position error, velocity, and integral
            float pitch_speed = compute_pitch_LQI_control(
                pitch_lqi_gains, 
                target_position, // Target position for pitch
                actual_positions[1] // Actual position (pitch)
            );
            motor2.setSpeed(static_cast<int16_t>(pitch_speed)); // Apply LQI control to motor2
            commanded_speeds[1] = pitch_speed; // Store commanded speed for telemetry
        } else if (axis_index == 0) { // Roll axis (LQR controller)
            float roll_lqr_gains[2] = {255.1886f, 4.0860f}; // Gains for position error and velocity
            float roll_speed = compute_roll_LQR_control(
                roll_lqr_gains, 
                target_position, // Target position for roll
                actual_positions[0] // Actual position (roll)
            );
            motor1.setSpeed(static_cast<int16_t>(roll_speed)); // Apply LQR control to motor1
            commanded_speeds[0] = roll_speed; // Store commanded speed for telemetry
        }

        // Delay for the specified time step
        delay(static_cast<unsigned long>(time_delay));
    }
}
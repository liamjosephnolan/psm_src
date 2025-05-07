#include "config.h"

// Function to compute the LQR control input
float compute_LQR_control(float* gains, float commanded_position, float actual_position) {
    static float previous_position = 0.0f; // Store the previous position
    static unsigned long previous_time = 0; // Store the previous time in microseconds
    static float filtered_velocity = 0.0f; // Store the filtered velocity
    const float alpha = 0.9f; // Smoothing factor for low-pass filter

    // Calculate delta time using micros() for higher precision
    unsigned long current_time = micros();
    float delta_time = (current_time - previous_time) / 1000000.0f; // Convert to seconds
    if (delta_time <= 0.0f) delta_time = 0.000001f; // Prevent division by zero

    // Calculate raw velocity using finite difference2
    float raw_velocity = (actual_position - previous_position) / delta_time;

    // Apply low-pass filter to smooth the velocity
    filtered_velocity = alpha * filtered_velocity + (1.0f - alpha) * raw_velocity;

    // Update the previous position and time
    previous_position = actual_position;
    previous_time = current_time;

    // Compute the error between commanded and actual position
    float position_error = commanded_position - actual_position;

    // Combine position error and filtered velocity into a state array
    float state[2] = {position_error, filtered_velocity}; // Position error and filtered velocity

    // Compute the LQR control input
    float control_input = 0.0f;
    for (int i = 0; i < 2; i++) { // Loop over the two states (position error and velocity)
        control_input -= gains[i] * state[i];
    }

    // Limit the control input to a maximum and minimum value
    if (control_input > 100) {
        control_input = 100; // Limit to maximum speed
    } else if (control_input < -100) {
        control_input = -100; // Limit to minimum speed
    }

    // Return the computed control input
    return control_input;
}
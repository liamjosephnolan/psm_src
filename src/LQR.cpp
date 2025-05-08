#include "config.h"

// Function to compute the LQR control input for the roll axis
float compute_roll_LQR_control(float* gains, float commanded_position, float actual_position) {
    static float previous_position_roll = 0.0f; // Store the previous position for roll
    static unsigned long previous_time_roll = 0; // Store the previous time for roll
    static float filtered_velocity_roll = 0.0f; // Store the filtered velocity for roll
    const float alpha = 0.9f; // Smoothing factor for low-pass filter

    // Calculate delta time using micros() for higher precision
    unsigned long current_time = micros();
    float delta_time = (current_time - previous_time_roll) / 1000000.0f; // Convert to seconds
    if (delta_time <= 0.0f) delta_time = 0.000001f; // Prevent division by zero

    // Calculate raw velocity using finite difference
    float raw_velocity = (actual_position - previous_position_roll) / delta_time;

    // Apply low-pass filter to smooth the velocity
    filtered_velocity_roll = alpha * filtered_velocity_roll + (1.0f - alpha) * raw_velocity;

    // Update the previous position and time
    previous_position_roll = actual_position;
    previous_time_roll = current_time;

    // Compute the error between commanded and actual position
    float position_error = commanded_position - actual_position;

    // Combine position error and filtered velocity into a state array
    float state[2] = {position_error, filtered_velocity_roll}; // Position error and filtered velocity

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

// Function to compute the LQR control input for the pitch axis
float compute_pitch_LQR_control(float* gains, float commanded_position, float actual_position) {
    static float previous_position_pitch = 0.0f; // Store the previous position for pitch
    static unsigned long previous_time_pitch = 0; // Store the previous time for pitch
    static float filtered_velocity_pitch = 0.0f; // Store the filtered velocity for pitch
    const float alpha = 0.9f; // Smoothing factor for low-pass filter

    // Calculate delta time using micros() for higher precision
    unsigned long current_time = micros();
    float delta_time = (current_time - previous_time_pitch) / 1000000.0f; // Convert to seconds
    if (delta_time <= 0.0f) delta_time = 0.000001f; // Prevent division by zero

    // Calculate raw velocity using finite difference
    float raw_velocity = (actual_position - previous_position_pitch) / delta_time;

    // Apply low-pass filter to smooth the velocity
    filtered_velocity_pitch = alpha * filtered_velocity_pitch + (1.0f - alpha) * raw_velocity;

    // Update the previous position and time
    previous_position_pitch = actual_position;
    previous_time_pitch = current_time;

    // Compute the error between commanded and actual position
    float position_error = commanded_position - actual_position;

    // Combine position error and filtered velocity into a state array
    float state[2] = {position_error, filtered_velocity_pitch}; // Position error and filtered velocity

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
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

// Function to compute the LQR control input for the pitch axis with feedforward control
float compute_pitch_LQR_control(float* gains, float commanded_position, float actual_position) {
    static float previous_position_pitch = 0.0f; // Store the previous position for pitch
    static unsigned long previous_time_pitch = 0; // Store the previous time for pitch
    static float previous_commanded_position = 0.0f; // Store the previous commanded position
    const float lambda = 0.0f; // Smoothing factor for feedforward control

    // System matrices
    const float A[2][2] = {{-4.3950, -2.3842}, {4.0000, 0.0}};
    const float B_dagger[2][2] = {{0.5, 0.0}, {0.0, 0.0}};

    // Calculate delta time using micros() for higher precision
    unsigned long current_time = micros();
    float delta_time = (current_time - previous_time_pitch) / 1000000.0f; // Convert to seconds
    if (delta_time <= 0.0f) delta_time = 0.000001f; // Prevent division by zero

    // Calculate raw velocity using finite difference
    float raw_velocity = (actual_position - previous_position_pitch) / delta_time;

    // Update the previous position and time
    previous_position_pitch = actual_position;
    previous_time_pitch = current_time;

    // Compute the error between commanded and actual position
    float position_error = commanded_position - actual_position;

    // Combine position error and raw velocity into a state array
    float state[2] = {position_error, raw_velocity}; // Position error and raw velocity

    // Compute the LQR control input
    float control_input = 0.0f;
    for (int i = 0; i < 2; i++) { // Loop over the two states (position error and velocity)
        control_input -= gains[i] * state[i];
    }

    // Calculate x_ref_dot (time derivative of commanded position)
    float x_ref_dot = (commanded_position - previous_commanded_position) / delta_time;

    // Update the previous commanded position
    previous_commanded_position = commanded_position;

    // Calculate feedforward control input
    float x_ref[2] = {commanded_position, x_ref_dot}; // Reference state
    float u_ff = 0.0f; // Feedforward control input

    for (int i = 0; i < 2; i++) {
        float Ax_ref = 0.0f;
        for (int j = 0; j < 2; j++) {
            Ax_ref += A[i][j] * x_ref[j]; // A * x_ref
        }
        u_ff += B_dagger[i][i] * (Ax_ref + x_ref_dot); // B_dagger * (A * x_ref + x_ref_dot)
    }

    // Combine LQR control input and feedforward control input
    control_input += lambda * u_ff;

    // Limit the control input to a maximum and minimum value
    if (control_input > 150) {
        control_input = 150; // Limit to maximum speed
    } else if (control_input < -150) {
        control_input = -150; // Limit to minimum speed
    }

    // Return the computed control input
    return control_input;
}
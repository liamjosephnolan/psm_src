#include "config.h"

// ----------------------
// Global Variables
// ----------------------

// Define global variables used in PID calculations
float dt = 0.01; // Time step for PID calculations
float prev_e[3] = {0, 0, 0}; // Previous error for each axis
float integral[3] = {0, 0, 0}; // Integral term for each axis
float control_values[3] = {0, 0, 0}; // Control values for each axis
float sat_control_values[3] = {0, 0, 0}; // Saturated control values for each axis
bool clamp_I[3] = {false, false, false}; // Clamping flags for each axis
int m_speed[3] = {0, 0, 0}; // Motor speeds for each axis

// ----------------------
// PID Update Function
// ----------------------

// Updates the PID controller for a given axis
float PIDupdate(float* target, int index, String mode, float kp, float ki, float kd) {
    // Function variables
    float current;
    int PID_select;
    float u; // Control signal
    float e; // Error
    int sign_e;
    int sign_u;
    float clamp_Lim_up;
    float clamp_Lim_low;
    int dir;
    int speed;

    // Select mode of controller ("P", "PI", "PD", or "PID")
    if (mode == "P") {
        PID_select = 0;
    } else if (mode == "PI") {
        PID_select = 1;
    } else if (mode == "PD") {
        PID_select = 2;
    } else if (mode == "PID") {
        PID_select = 3;
    } else {
        PID_select = 0; // Default to P-mode
    }

    // Choose parameters based on index
    switch (index) {
        case 0: // Axis 1
            current = Ax1toAngle(Enc1.read());
            clamp_Lim_up = 32;
            clamp_Lim_low = -32;
            break;
        case 1: // Axis 2
            current = Ax2toAngle(Enc2.read());
            clamp_Lim_up = 40;
            clamp_Lim_low = -40;
            break;
        case 2: // Axis 3
            current = Ax3toAngle(Enc3.read());
            clamp_Lim_up = 7;
            clamp_Lim_low = -7;
            break;
        default:
            current = 0;
            clamp_Lim_up = 0;
            clamp_Lim_low = 0;
            break;
    }

    // Calculate error
    e = *target - current;

    // PID controller state machine
    switch (PID_select) {
        case 0: // P-mode
            u = kp * e;
            break;

        case 1: // PI-mode
            if (!clamp_I[index]) {
                integral[index] += e * dt; // Integrator
                u = kp * e + ki * integral[index];
            } else {
                u = kp * e;
            }
            break;

        case 2: // PD-mode
            u = kp * e + kd * (e - prev_e[index]) / dt;
            break;

        case 3: // PID-mode
            if (!clamp_I[index]) {
                integral[index] += e * dt; // Integrator
                u = kp * e + ki * integral[index] + kd * (e - prev_e[index]) / dt;
            } else {
                u = kp * e + kd * (e - prev_e[index]) / dt;
            }
            break;
    }

    // Update previous error
    prev_e[index] = e;

    // Check if clamping is required
    control_values[index] = u;
    sat_control_values[index] = constrain(u, clamp_Lim_low, clamp_Lim_up); // Prevent windup
    sign_u = (u < 0) ? -1 : 1;
    sign_e = (e < 0) ? -1 : 1;
    clamp_I[index] = (sign_u == sign_e) && (control_values[index] != sat_control_values[index]);

    // Determine direction
    dir = (u < 0) ? 1 : -1;

    // Determine speed
    speed = (int)fabs(u);

 
    
    // Limit speed to maximum value
    if (speed > 100) {
        speed = 100;
    }

    // Minimum speed for Axis 3
    if ((speed < 8) && (index == 2)) {
        speed = 8;
    }

    m_speed[index] = dir * speed; // For evaluation

    // Set motor speed
    motor[index].setSpeed(dir * speed);

    // Publish Speed
    commanded_speeds[index] = static_cast<double>(dir* speed);

    // Return voltage as dir * speed
    return dir * speed;
}
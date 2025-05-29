#include "config.h"

// Gain tables for Roll (3x3)
const double Kp_roll_table[3][3] = {
    {254.22, 254.21, 243.35},  // Pitch -15
    {255.41, 249.61, 246.55},  // Pitch 0
    {257.05, 254.74, 248.95}   // Pitch 15
};

const double Kd_roll_table[3][3] = {
    {2.1527, 1.5829, 0.53766}, // Pitch -15
    {1.7854, 1.2588, 0.52810}, // Pitch 0
    {2.4579, 1.5788, 0.62543}  // Pitch 15
};

const double Ki_roll_table[3][3] = {
    {0.0, 0.0, 0.0},           // Pitch -15
    {0.0, 0.0, 0.0},           // Pitch 0
    {0.0, 0.0, 0.0}            // Pitch 15
};

// Gain tables for Pitch (3x3)
const double Kp_pitch_table[3][3] = {
    {135.14, 155.14, 135.14},  // Roll -15
    {135.37, 155.37, 135.37},  // Roll 0
    {115.37, 135.37, 115.37}   // Roll 15
};

const double Kd_pitch_table[3][3] = {
    {0.242221, 0.2221, .24221}, // Roll -15
    {0.24671, 0.44671, 0.24671}, // Roll 0
    {0.14671, 0.24671, 0.14671}  // Roll 15
};

const double Ki_pitch_table[3][3] = {
    {0.11242, 0.11242, 0.11242}, // Roll -15
    {0.11649, 0.11649, 0.11649}, // Roll 02
    {0.11649, 0.11649, 0.11649}  // Roll 15
};

// Available pitch and roll angle values
const double pitch_values[3] = {-20.0, 0.0, 20.0};
const double roll_values[3]  = {-15.0, 0.0, 15.0};

// Helper function to perform linear interpolation
double interpolate(double x, double x0, double x1, double y0, double y1) {
    return y0 + (y1 - y0) * ((x - x0) / (x1 - x0));
}

// Gain scheduling function for Roll
Gains getRollGains(double roll_angle, double pitch_angle) {
    // Find the nearest pitch indices
    int pitch_idx_low = 0;
    int pitch_idx_high = 1;
    for (int i = 0; i < 2; ++i) {
        if (pitch_angle >= pitch_values[i] && pitch_angle <= pitch_values[i + 1]) {
            pitch_idx_low = i;
            pitch_idx_high = i + 1;
            break;
        }
    }

    // Find the nearest roll indices
    int roll_idx_low = 0;
    int roll_idx_high = 1;
    for (int i = 0; i < 2; ++i) {
        if (roll_angle >= roll_values[i] && roll_angle <= roll_values[i + 1]) {
            roll_idx_low = i;
            roll_idx_high = i + 1;
            break;
        }
    }

    // Interpolate Kp
    double Kp_low = interpolate(roll_angle, roll_values[roll_idx_low], roll_values[roll_idx_high],
                                Kp_roll_table[pitch_idx_low][roll_idx_low], Kp_roll_table[pitch_idx_low][roll_idx_high]);
    double Kp_high = interpolate(roll_angle, roll_values[roll_idx_low], roll_values[roll_idx_high],
                                 Kp_roll_table[pitch_idx_high][roll_idx_low], Kp_roll_table[pitch_idx_high][roll_idx_high]);
    double Kp = interpolate(pitch_angle, pitch_values[pitch_idx_low], pitch_values[pitch_idx_high], Kp_low, Kp_high);

    // Interpolate Kd
    double Kd_low = interpolate(roll_angle, roll_values[roll_idx_low], roll_values[roll_idx_high],
                                Kd_roll_table[pitch_idx_low][roll_idx_low], Kd_roll_table[pitch_idx_low][roll_idx_high]);
    double Kd_high = interpolate(roll_angle, roll_values[roll_idx_low], roll_values[roll_idx_high],
                                 Kd_roll_table[pitch_idx_high][roll_idx_low], Kd_roll_table[pitch_idx_high][roll_idx_high]);
    double Kd = interpolate(pitch_angle, pitch_values[pitch_idx_low], pitch_values[pitch_idx_high], Kd_low, Kd_high);

    // Return the interpolated gains
    Gains gains;
    gains.Kp = Kp;
    gains.Kd = Kd;
    gains.Ki = 0.0; // Ki is always 0 for roll
    return gains;
}

// Gain scheduling function for Pitch
Gains getPitchGains(double pitch_angle, double roll_angle) {
    // Find the nearest pitch indices
    int pitch_idx_low = 0;
    int pitch_idx_high = 1;
    for (int i = 0; i < 2; ++i) {
        if (pitch_angle >= pitch_values[i] && pitch_angle <= pitch_values[i + 1]) {
            pitch_idx_low = i;
            pitch_idx_high = i + 1;
            break;
        }
    }

    // Find the nearest roll indices
    int roll_idx_low = 0;
    int roll_idx_high = 1;
    for (int i = 0; i < 2; ++i) {
        if (roll_angle >= roll_values[i] && roll_angle <= roll_values[i + 1]) {
            roll_idx_low = i;
            roll_idx_high = i + 1;
            break;
        }
    }

    // Interpolate Kp
    double Kp_low = interpolate(roll_angle, roll_values[roll_idx_low], roll_values[roll_idx_high],
                                Kp_pitch_table[pitch_idx_low][roll_idx_low], Kp_pitch_table[pitch_idx_low][roll_idx_high]);
    double Kp_high = interpolate(roll_angle, roll_values[roll_idx_low], roll_values[roll_idx_high],
                                 Kp_pitch_table[pitch_idx_high][roll_idx_low], Kp_pitch_table[pitch_idx_high][roll_idx_high]);
    double Kp = interpolate(pitch_angle, pitch_values[pitch_idx_low], pitch_values[pitch_idx_high], Kp_low, Kp_high);

    // Interpolate Kd
    double Kd_low = interpolate(roll_angle, roll_values[roll_idx_low], roll_values[roll_idx_high],
                                Kd_pitch_table[pitch_idx_low][roll_idx_low], Kd_pitch_table[pitch_idx_low][roll_idx_high]);
    double Kd_high = interpolate(roll_angle, roll_values[roll_idx_low], roll_values[roll_idx_high],
                                 Kd_pitch_table[pitch_idx_high][roll_idx_low], Kd_pitch_table[pitch_idx_high][roll_idx_high]);
    double Kd = interpolate(pitch_angle, pitch_values[pitch_idx_low], pitch_values[pitch_idx_high], Kd_low, Kd_high);

    // Interpolate Ki
    double Ki_low = interpolate(roll_angle, roll_values[roll_idx_low], roll_values[roll_idx_high],
                                Ki_pitch_table[pitch_idx_low][roll_idx_low], Ki_pitch_table[pitch_idx_low][roll_idx_high]);
    double Ki_high = interpolate(roll_angle, roll_values[roll_idx_low], roll_values[roll_idx_high],
                                 Ki_pitch_table[pitch_idx_high][roll_idx_low], Ki_pitch_table[pitch_idx_high][roll_idx_high]);
    double Ki = interpolate(pitch_angle, pitch_values[pitch_idx_low], pitch_values[pitch_idx_high], Ki_low, Ki_high);

    // Return the interpolated gains
    Gains gains;
    gains.Kp = Kp;
    gains.Kd = Kd;
    gains.Ki = Ki;
    return gains;
}
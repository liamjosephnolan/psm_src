#include "config.h"

// ===============================================
//   Gain Tables for LQR-based Gain Scheduling
//   -------------------------------------------
//   - Tables are 3x3 grids storing precomputed
//     PID gains for each roll/pitch combination.
//   - Both Roll and Pitch controllers are scheduled
//     using the current pitch and roll angles.
// 
//   - Row index 0 → pitch = -20
//   - Row index 1 → pitch = 0
//   - Row index 2 → pitch = +20
//
//   - Column index 0 → roll = -15
//   - Column index 1 → roll = 0
//   - Column index 2 → roll = +15
//
//   Access pattern: table[pitch_idx][roll_idx]
//
// ===============================================

// === Gain tables for Roll controller (uses pitch + roll angles) ===
// const double Kp_roll_table[3][3] = {
//     {254.22, 254.21, 243.35},  // pitch = -20, roll = [-15, 0, 15]
//     {255.41, 249.61, 246.55},  // pitch =  0
//     {257.05, 254.74, 248.95}   // pitch = +20
// };

// const double Kd_roll_table[3][3] = {
//     {2.1527, 1.5829, 0.53766}, // pitch = -20
//     {1.7854, 1.2588, 0.52810}, // pitch =  0
//     {2.4579, 1.5788, 0.62543}  // pitch = +20
// };

const double Kp_roll_table[3][3] = {
    {135.41, 135.41, 137.91},  // pitch = -20, roll = [-15, 0, 15]
    {136.23, 132.23, 130.12},  // pitch =  0
    {137.36, 135.77, 131.78}   // pitch = +20
};

const double Kd_roll_table[3][3] = {
    {0.6527, 1.0829, 0.43766}, // pitch = -20
    {0.6854, 0.7588, 0.4810}, // pitch =  0
    {0.6579, 1.0788, 0.42543}  // pitch = +20
};

const double Ki_roll_table[3][3] = {
    {0.0, 0.0, 0.0},           // pitch = -20
    {0.0, 0.0, 0.0},           // pitch =  0
    {0.0, 0.0, 0.0}            // pitch = +20
};

// === Gain tables for Pitch controller (uses pitch + roll angles) ===
const double Kp_pitch_table[3][3] = {
    {185.37, 185.37, 185.37},  // pitch = -20, roll = [-15, 0, 15]
    {165.37, 165.37, 165.37},  // pitch =  0
    {165.37, 165.37, 165.37}   // pitch = +20
};

const double Kd_pitch_table[3][3] = {
    {0.3671, 0.3671, 0.3671}, // pitch = -20
    {0.3671, 0.3671, 0.3671}, // pitch =  0
    {0.3671, 0.3671, 0.3671}  // pitch = +20
};

const double Ki_pitch_table[3][3] = {
    {0.185649, 0.185649, 0.185649}, // pitch = -20
    {0.155649, 0.155649, 0.155649}, // pitch =  0
    {0.155649, 0.155649, 0.155649}  // pitch = +20
};

// Angle lookup vectors (for interpolation grid)
const double pitch_values[3] = {-20.0, 0.0, 20.0};
const double roll_values[3]  = {-15.0, 0.0, 15.0};

// === Helper function: Linear interpolation between two values ===
double interpolate(double x, double x0, double x1, double y0, double y1) {
    return y0 + (y1 - y0) * ((x - x0) / (x1 - x0));
}

// === Gain scheduling function for Roll controller ===
Gains getRollGains(double roll_angle, double pitch_angle) {
    // Identify bounding pitch indices
    int pitch_idx_low = 0, pitch_idx_high = 1;
    for (int i = 0; i < 2; ++i) {
        if (pitch_angle >= pitch_values[i] && pitch_angle <= pitch_values[i + 1]) {
            pitch_idx_low = i;
            pitch_idx_high = i + 1;
            break;
        }
    }

    // Identify bounding roll indices
    int roll_idx_low = 0, roll_idx_high = 1;
    for (int i = 0; i < 2; ++i) {
        if (roll_angle >= roll_values[i] && roll_angle <= roll_values[i + 1]) {
            roll_idx_low = i;
            roll_idx_high = i + 1;
            break;
        }
    }

    // Interpolate Kp
    double Kp_low  = interpolate(roll_angle, roll_values[roll_idx_low], roll_values[roll_idx_high],
                                 Kp_roll_table[pitch_idx_low][roll_idx_low], Kp_roll_table[pitch_idx_low][roll_idx_high]);
    double Kp_high = interpolate(roll_angle, roll_values[roll_idx_low], roll_values[roll_idx_high],
                                 Kp_roll_table[pitch_idx_high][roll_idx_low], Kp_roll_table[pitch_idx_high][roll_idx_high]);
    double Kp = interpolate(pitch_angle, pitch_values[pitch_idx_low], pitch_values[pitch_idx_high], Kp_low, Kp_high);

    // Interpolate Kd
    double Kd_low  = interpolate(roll_angle, roll_values[roll_idx_low], roll_values[roll_idx_high],
                                 Kd_roll_table[pitch_idx_low][roll_idx_low], Kd_roll_table[pitch_idx_low][roll_idx_high]);
    double Kd_high = interpolate(roll_angle, roll_values[roll_idx_low], roll_values[roll_idx_high],
                                 Kd_roll_table[pitch_idx_high][roll_idx_low], Kd_roll_table[pitch_idx_high][roll_idx_high]);
    double Kd = interpolate(pitch_angle, pitch_values[pitch_idx_low], pitch_values[pitch_idx_high], Kd_low, Kd_high);

    // Ki is fixed to 0 for roll controller
    Gains gains;
    gains.Kp = Kp;
    gains.Kd = Kd;
    gains.Ki = 0.0;
    return gains;
}

// === Gain scheduling function for Pitch controller ===
Gains getPitchGains(double pitch_angle, double roll_angle) {
    // Identify bounding pitch indices
    int pitch_idx_low = 0, pitch_idx_high = 1;
    for (int i = 0; i < 2; ++i) {
        if (pitch_angle >= pitch_values[i] && pitch_angle <= pitch_values[i + 1]) {
            pitch_idx_low = i;
            pitch_idx_high = i + 1;
            break;
        }
    }

    // Identify bounding roll indices
    int roll_idx_low = 0, roll_idx_high = 1;
    for (int i = 0; i < 2; ++i) {
        if (roll_angle >= roll_values[i] && roll_angle <= roll_values[i + 1]) {
            roll_idx_low = i;
            roll_idx_high = i + 1;
            break;
        }
    }

    // Interpolate Kp
    double Kp_low  = interpolate(roll_angle, roll_values[roll_idx_low], roll_values[roll_idx_high],
                                 Kp_pitch_table[pitch_idx_low][roll_idx_low], Kp_pitch_table[pitch_idx_low][roll_idx_high]);
    double Kp_high = interpolate(roll_angle, roll_values[roll_idx_low], roll_values[roll_idx_high],
                                 Kp_pitch_table[pitch_idx_high][roll_idx_low], Kp_pitch_table[pitch_idx_high][roll_idx_high]);
    double Kp = interpolate(pitch_angle, pitch_values[pitch_idx_low], pitch_values[pitch_idx_high], Kp_low, Kp_high);

    // Interpolate Kd
    double Kd_low  = interpolate(roll_angle, roll_values[roll_idx_low], roll_values[roll_idx_high],
                                 Kd_pitch_table[pitch_idx_low][roll_idx_low], Kd_pitch_table[pitch_idx_low][roll_idx_high]);
    double Kd_high = interpolate(roll_angle, roll_values[roll_idx_low], roll_values[roll_idx_high],
                                 Kd_pitch_table[pitch_idx_high][roll_idx_low], Kd_pitch_table[pitch_idx_high][roll_idx_high]);
    double Kd = interpolate(pitch_angle, pitch_values[pitch_idx_low], pitch_values[pitch_idx_high], Kd_low, Kd_high);

    // Interpolate Ki
    double Ki_low  = interpolate(roll_angle, roll_values[roll_idx_low], roll_values[roll_idx_high],
                                 Ki_pitch_table[pitch_idx_low][roll_idx_low], Ki_pitch_table[pitch_idx_low][roll_idx_high]);
    double Ki_high = interpolate(roll_angle, roll_values[roll_idx_low], roll_values[roll_idx_high],
                                 Ki_pitch_table[pitch_idx_high][roll_idx_low], Ki_pitch_table[pitch_idx_high][roll_idx_high]);
    double Ki = interpolate(pitch_angle, pitch_values[pitch_idx_low], pitch_values[pitch_idx_high], Ki_low, Ki_high);

    // Return interpolated pitch gains
    Gains gains;
    gains.Kp = Kp;
    gains.Kd = Kd;
    gains.Ki = Ki;
    return gains;
}

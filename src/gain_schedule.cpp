#include "config.h"

// Gain tables (3x3)
const double Kp_table[3][3] = {
    {254.22, 254.21, 243.35},  // Pitch -15
    {255.41, 249.61, 246.55},  // Pitch 0
    {257.05, 254.74, 248.95}   // Pitch 15
};

const double Kd_table[3][3] = {
    {2.1527, 1.5829, 0.53766}, // Pitch -15
    {1.7854, 1.2588, 0.52810}, // Pitch 0
    {2.4579, 1.5788, 0.62543}  // Pitch 15
};

// Available pitch and roll angle values
const double pitch_values[3] = {-15.0, 0.0, 15.0};
const double roll_values[3]  = {-15.0, 0.0, 15.0};

// Function to find the nearest index in an array
int nearestIndex(const double* array, int size, double value) {
    int nearest = 0;
    double min_diff = std::abs(array[0] - value);
    for (int i = 1; i < size; ++i) {
        double diff = std::abs(array[i] - value);
        if (diff < min_diff) {
            min_diff = diff;
            nearest = i;
        }
    }
    return nearest;
}

// Gain scheduling function
Gains getRollGains(double roll_angle, double pitch_angle) {
    int pitch_idx = nearestIndex(pitch_values, 3, pitch_angle);
    int roll_idx  = nearestIndex(roll_values, 3, roll_angle);

    Gains gains;
    gains.Kp = Kp_table[pitch_idx][roll_idx];
    gains.Kd = Kd_table[pitch_idx][roll_idx];

    return gains;
}
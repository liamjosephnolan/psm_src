#include "config.h"

// Static variables to store previous filtered values for each encoder
static float filtered_values[3] = {0.0f, 0.0f, 0.0f};

// Function to read and filter encoder signals
float read_filtered_encoder(int index) {
    // Read raw encoder value based on the index
    long raw_value = 0;
    switch (index) {
        case 0: // Roll
            raw_value = Enc1.read();
            break;
        case 1: // Pitch
            raw_value = Enc2.read();
            break;
        case 2: // Insertion
            raw_value = Enc3.read();
            break;
        default:
            return 0.0f; // Invalid index
    }

    // Apply low-pass filter (Exponential Moving Average)
    float filtered_value = LPF_ALPHA * raw_value + (1.0f - LPF_ALPHA) * filtered_values[index];

    // Update the stored filtered value
    filtered_values[index] = filtered_value;

    // Return the filtered value
    return filtered_value;
}
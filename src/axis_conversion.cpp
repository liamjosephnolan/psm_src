#include "config.h"

// ----------------------
// Global Variables
// ----------------------

// Define the global variable for encoder resolution
float res_avago = 2000; // Counts per revolution

// ----------------------
// Axis Conversion Functions
// ----------------------

// Converts encoder counts to angle (degrees) for Axis 1
float Ax1toAngle(long count) {
    float trans = 20.000; // Gear reduction ratio
    return ((float)count / (res_avago * trans) * 360.0) - 30;
}

// Converts encoder counts to angle (degrees) for Axis 2
float Ax2toAngle(long count) {
    float trans = 13.333; // Gear reduction ratio
    return ((-1) * ((float)count / (res_avago * trans) * 360.0)) - 30;
}

// Converts encoder counts to position (mm) for Axis 3
float Ax3toAngle(long count) {
    float D = 19.10; // Diameter of the pulley (mm)
    float ref = 360; // Reference angle (degrees)
    return (PI * D * (float)count * res_avago) / ref;
}

// Converts angle (degrees) to encoder counts for Axis 1
int Ax1toCounts(float angle) {
    float trans = 20.000; // Gear reduction ratio
    return int(-1.0 * angle * trans / res_avago);
}

// Converts angle (degrees) to encoder counts for Axis 2
int Ax2toCounts(float angle) {
    float trans = 13.333; // Gear reduction ratio
    return int(-1.0 * angle * trans / res_avago);
}

// Converts position (mm) to encoder counts for Axis 3
int Ax3toCounts(float pos) {
    float D = 19.10; // Diameter of the pulley (mm)
    float ref = 360; // Reference angle (degrees)
    return int(1.0 * (pos * ref) / (PI * D * res_avago));
}
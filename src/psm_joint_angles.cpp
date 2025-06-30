#include "config.h"

// ----------------------
// Helper Functions
// ----------------------

// Constrains a value between a minimum and maximum
double constrain_value(double value, double min_val, double max_val) {
    return (value < min_val) ? min_val : (value > max_val) ? max_val : value;
}

// ----------------------
// Joint Angle Computation
// ----------------------

// Computes the joint angles (q1, q2, q3) for the PSM based on the target position
JointAngles computePSMJointAngles(double x_p, double y_p, double z_p) {
    // Apply the transformations
    double x = -x_p * 1000;      // Invert X
    double y = z_p * 1000;       // Swap Y and Z
    double z = y_p * 1000;


    JointAngles angles;

    // Compute insertion distance (q3) as the Euclidean distance
    angles.q3 = static_cast<float>(std::sqrt(x * x + y * y + z * z));

    // Compute pitch angle (q2) in degrees and constrain
    double pitch = (std::atan2(x, y) * 180.0 / M_PI)/4.0;
    pitch = constrain_value(pitch, -15.0, 15.0);
    angles.q2 = static_cast<float>(pitch);

    // Compute yaw angle (q1) in degrees and constrain
    double yaw = (std::atan2(z, y) * 180.0 / M_PI)/4;
    yaw = constrain_value(yaw, -20.0, 20.0);
    angles.q1 = static_cast<float>(yaw);

    // telemetry publishing
    commanded_positions[0] = angles.q1;
    commanded_positions[1] = angles.q2;
    commanded_positions[2] = angles.q3;

    return angles;
}
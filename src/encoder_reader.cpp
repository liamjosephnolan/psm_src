#include "config.h" // Include shared pin definitions

// ----------------------
// Encoder Objects
// ----------------------

// Define encoder objects (global)
Encoder Enc1(ENC1_B, ENC1_A);
Encoder Enc2(ENC2_B, ENC2_A);
Encoder Enc3(ENC3_B, ENC3_A);

// ----------------------
// Encoder Reading Function
// ----------------------

// Reads encoder data and populates the ROS 2 Int32MultiArray message
void read_encoder_data(std_msgs__msg__Int32MultiArray *msg) {
    msg->data.data[0] = Enc1.read();
    msg->data.data[1] = Enc2.read();
    msg->data.data[2] = Enc3.read();
    msg->data.size = 3; // Ensure the size matches the number of encoders
}
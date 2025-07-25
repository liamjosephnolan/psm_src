#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <cmath>

// ----------------------
// Macros for Error Handling
// ----------------------
#define RCCHECK(fn) { rcl_ret_t rc = fn; if (rc != RCL_RET_OK) error_loop(); }
#define RCSOFTCHECK(fn) { rcl_ret_t rc = fn; if (rc != RCL_RET_OK) {} }

// ----------------------
// Arduino and Hardware Libraries
// ----------------------
#include <Encoder.h>
#include <Servo.h>
#include "CytronMotorDriver.h"

// ----------------------
// ROS 2 and micro-ROS Libraries
// ----------------------
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <sensor_msgs/msg/joint_state.h>
#include <std_msgs/msg/string.h>
#include <geometry_msgs/msg/point_stamped.h>
#include <rosidl_runtime_c/string_functions.h>
#include <rmw/qos_profiles.h>  // for rmw_qos_profile_default


// ----------------------
// C/C++ Standard Libraries
// ----------------------
#include <cstdio>
#include <cstring>
#include <vector>
#include <utility>
#include <fstream>
#include <sstream>

// -----Joint Definitions-----
#define JOINT_COUNT 7

// ----------------------
// Pin Definitions
// ----------------------
#define ENC1_A 0
#define ENC1_B 1
#define ENC2_A 2
#define ENC2_B 3
#define ENC3_A 4
#define ENC3_B 5

#define DC1_PWM 6
#define DC2_PWM 7
#define DC3_PWM 8

#define DC1_DIR 24
#define DC2_DIR 25
#define DC3_DIR 26

#define LS1_NC 22
#define LS1_NO 23
#define LS2_NC 20
#define LS2_NO 21
#define LS3_NC 18
#define LS3_NO 19

#define SERVO1_PIN 9   // Right grasper finger
#define SERVO2_PIN 10  // Left grasper finger
#define SERVO3_PIN 11  // Wrist roll
#define SERVO4_PIN 12  // Wrist pitch

#define G0_MIN 0    // TODO: Fix G0 angle stuff
#define G0_MAX 75
#define G1_MIN -40
#define G1_MAX 40
#define G2_MIN -40
#define G2_MAX 40
#define G3_MIN -125 
#define G3_MAX 125

#define SERVO_ANGLE_MIN -360
#define SERVO_ANGLE_MAX 360

#define JOINT_NAME_MAX 10
#define NAME_LENGTH_MAX 30

// Low-pass filter constant (tunable)
#define LPF_ALPHA 1.0f // Alpha value for the low-pass filter (0 < LPF_ALPHA <= 1)


// ----------------------
// Sine Wave Parameters
// ----------------------
// Amplitudes (adjust units as needed: radians for angles, mm for insertion)
#define ROLL_SINE_AMPLITUDE 20.0f   // Example amplitude for Roll
#define PITCH_SINE_AMPLITUDE 15.0f  // Example amplitude for Pitch
#define INSERTION_SINE_AMPLITUDE 100.0f // Example amplitude for Insertion

// Periods in seconds
#define ROLL_SINE_PERIOD 3.0f    // Example period for Roll in seconds
#define PITCH_SINE_PERIOD 3.0f   // Example period for Pitch in seconds
#define INSERTION_SINE_PERIOD 5.0f // Example period for Insertion in seconds


// ----------------------
// Global/External Variables
// ----------------------

// Encoder resolution (counts per revolution)
extern float res_avago;

// Time step for PID calculations
extern float dt;

// Arrays for PID state variables
extern float prev_e[3];
extern float integral[3];
extern float control_values[3];
extern float sat_control_values[3];
extern bool clamp_I[3];
extern int m_speed[3];

// Voltage variables for motors
extern int16_t roll_voltage;
extern int16_t pitch_voltage;

// Motor objects
extern CytronMD motor1;
extern CytronMD motor2;
extern CytronMD motor3;
extern CytronMD motor[3];

// Encoder objects
extern Encoder Enc1;
extern Encoder Enc2;
extern Encoder Enc3;

// ROS 2 global objects
extern std_msgs__msg__String debug_msg;
extern rcl_publisher_t debug_publisher;

// telemtry message variables
extern double actual_positions[3];
extern double commanded_positions[3];
extern double commanded_speeds[3];

// ----------------------
// Struct Definitions
// ----------------------

// Define a struct to hold the joint angles (use float to match PIDupdate)
struct JointAngles {
    float q1;  // Yaw
    float q2;  // Pitch
    float q3;  // Insertion
};

// Struct to hold the gains
struct Gains {
    double Kp;
    double Kd;
    double Ki;
};

// ----------------------
// Trajectory Following
// ----------------------
// Struct for trajectory points
struct TrajPoint {
    double time;
    double x;
    double y;
    double z;
};

// Extern declarations for trajectory data
extern std::vector<TrajPoint> trajectory;
extern void parse_traj_file();

// ----------------------
// Function Declarations
// ----------------------

// Axis conversion functions
extern float Ax1toAngle(long count);
extern float Ax2toAngle(long count);
extern float Ax3toAngle(long count);

// Declare the LQR function
float compute_roll_LQR_control(float* gains, float commanded_position, float actual_position);
float compute_pitch_LQR_control(float* gains, float commanded_position, float actual_position);
float compute_pitch_LQI_control(float* gains, float commanded_position, float actual_position);

// Function declarations
Gains getRollGains(double roll_angle, double pitch_angle);
Gains getPitchGains(double pitch_angle, double roll_angle); // Declare getPitchGains

// Helper functions
extern void publish_debug_message(const char *message);
extern int map_gimbal_to_servo(double gimbal_angle, double gimbal_min, double gimbal_max, double max_angle, double min_angle);

// Joint angle computation
extern JointAngles computePSMJointAngles(double x_p, double y_p, double z_p);

// Error handling
extern void error_loop();

// Encoder data reading
extern void read_encoder_data(std_msgs__msg__Int32MultiArray *msg);

// Motor control
extern void home_motors();
extern float PIDupdate(float *target, int index, String mode, float kp, float ki, float kd);

// Declare the function so it can be called from anywhere
float read_filtered_encoder(int index);

// External function declaration
void ramp(float current_angle, float target_angle, int axis_index, float ramp_time);

#endif // CONFIG_H
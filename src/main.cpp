#include <Arduino.h>
#include "config.h"

// ----------------------
// Global ROS 2 Objects
// ----------------------
rcl_subscription_t joint_state_subscriber;
rcl_subscription_t target_pose_subscriber;
rcl_publisher_t sensor_data_publisher;
rcl_publisher_t debug_publisher;
rcl_publisher_t joint_telemetry_publisher;  // Telemetry publisher
rcl_publisher_t gains_publisher;

sensor_msgs__msg__JointState received_joint_state;
sensor_msgs__msg__JointState target_pose_msg;
std_msgs__msg__Int32MultiArray sensor_data_msg;
std_msgs__msg__String debug_msg;
sensor_msgs__msg__JointState joint_telemetry_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// ----------------------
// Servo and Motor Control
// ---------------------- 
Servo servo1, servo2, servo3, servo4;

CytronMD motor1(PWM_DIR, DC1_PWM, DC1_DIR);
CytronMD motor2(PWM_DIR, DC2_PWM, DC2_DIR);
CytronMD motor3(PWM_DIR, DC3_PWM, DC3_DIR);
CytronMD motor[3] = {motor1, motor2, motor3};

// Servo Calibration Arrays
int servo_off[4] = {100, 97, 90, 92};
int servo_val[4] = {0, 0, 0, 0};

// Initialize telemetry message variables
double actual_positions[3] = {0.0, 0.0, 0.0};
double commanded_positions[3] = {0.0, 0.0, 0.0};
double commanded_speeds[3] = {0.0, 0.0, 0.0};

// Voltage variables for motors
int16_t roll_voltage = 0;
int16_t pitch_voltage = 0;

// ----------------------
// Telemetry Static Buffers
// ----------------------
rosidl_runtime_c__String joint_names[JOINT_COUNT];
char joint_names_data[JOINT_COUNT][12];  // Enough room for "insertion" + null terminator

double joint_positions[JOINT_COUNT];
double joint_velocities[JOINT_COUNT];
double joint_efforts[JOINT_COUNT];

int32_t sensor_data_array[3];
char debug_data_buffer[128];

// ----------------------
// Coupling Matrix Function
// ----------------------
void calculate_disk_movements(float roll, float pitch, float yaw, float grip, float* disk_movements) {
    // Coupling matrix
    const float coupling_matrix[4][4] = {
        {-1.56323325,  0.0,          0.0,          0.0        }, // Disk 1
        { 0.0,         1.01857984,  -0.830634273,  0.0        }, // Disk 2
        { 0.0,         0.0,          0.608862987, -1.21772597}, // Disk 3
        { 0.0,         0.0,          0.608862987,  1.21772597}  // Disk 4
    };

    // Desired tool movements
    const float tool_movements[4] = {roll, pitch, yaw, grip};

    // Calculate disk movements using matrix multiplication
    for (int i = 0; i < 4; i++) {
        disk_movements[i] = 0.0;
        for (int j = 0; j < 4; j++) {
            disk_movements[i] += coupling_matrix[i][j] * tool_movements[j];
        }
    }
}

// ----------------------
// ROS Callbacks
// ----------------------

// Joint state callback for grasper joints
void joint_state_callback(const void *msgin) {
    publish_debug_message("Received joint state message");
    const sensor_msgs__msg__JointState *msg = (const sensor_msgs__msg__JointState *)msgin;

    if (msg->position.size < 7 || msg->position.data == NULL) {
        publish_debug_message("[WARN] Invalid joint state message.");
        return;
    }

    // Extract joint positions
    float g0 = msg->position.data[6]; // Grasper pinch
    float g1 = msg->position.data[5]; // Grasper tilt
    float g2 = msg->position.data[4]; // Grasper pitch
    float g3 = msg->position.data[3]; // Grasper roll

    // Calculate disk movements
    float disk_movements[4];
    calculate_disk_movements(g3, g2, g1, g0, disk_movements);

    // Account for servo offsets
    disk_movements[0] += servo_off[3];
    disk_movements[1] += servo_off[2];
    disk_movements[2] += servo_off[0];
    disk_movements[3] += servo_off[1];

    // Command servos
    servo1.write(disk_movements[2]);
    servo2.write(disk_movements[3]);
    servo3.write(disk_movements[1]);
    servo4.write(disk_movements[0]);

    // Store commanded servo values in telemetry effort field
    joint_telemetry_msg.effort.data[3] = disk_movements[2];
    joint_telemetry_msg.effort.data[4] = disk_movements[3];
    joint_telemetry_msg.effort.data[5] = disk_movements[1];
    joint_telemetry_msg.effort.data[6] = disk_movements[0];
}

// Target pose callback (now receiving a JointState message with positions x, y, z)
void target_pose_callback(const void *msgin) {
    publish_debug_message("Received target pose message");
    const sensor_msgs__msg__JointState *msg = (const sensor_msgs__msg__JointState *)msgin;


    // Store x, y, z positions in telemetry effort fields
    joint_telemetry_msg.effort.data[0] = msg->position.data[0]; // x
    joint_telemetry_msg.effort.data[1] = msg->position.data[1]; // y
    joint_telemetry_msg.effort.data[2] = msg->position.data[2]; // z

    double x_target = msg->position.data[0];
    double y_target = msg->position.data[1];
    double z_target = msg->position.data[2];

    JointAngles angles = computePSMJointAngles(x_target, y_target, z_target);

    joint_telemetry_msg.position.data[0] = angles.q1; // Roll
    joint_telemetry_msg.position.data[1] = angles.q2; // Pitch
    joint_telemetry_msg.position.data[2] = angles.q3; // Insertion
    
}

// ----------------------
// Telemetry Functions
// ----------------------
void init_joint_telemetry_message() {
    // Initialize joint names
    joint_telemetry_msg.name.data = joint_names;
    joint_telemetry_msg.name.size = JOINT_COUNT;
    joint_telemetry_msg.name.capacity = JOINT_COUNT;

    const char* names[JOINT_COUNT] = {"roll", "pitch", "insertion", "servo1", "servo2", "servo3", "servo4"};

    for (size_t i = 0; i < JOINT_COUNT; i++) {
        rosidl_runtime_c__String__init(&joint_names[i]);
        joint_names[i].data = joint_names_data[i];
        joint_names[i].size = strlen(names[i]);
        joint_names[i].capacity = sizeof(joint_names_data[i]);
        strncpy(joint_names[i].data, names[i], joint_names[i].capacity);
    }

    // Set telemetry arrays
    joint_telemetry_msg.position.data = joint_positions;
    joint_telemetry_msg.position.size = JOINT_COUNT;
    joint_telemetry_msg.position.capacity = JOINT_COUNT;

    joint_telemetry_msg.velocity.data = joint_velocities;
    joint_telemetry_msg.velocity.size = JOINT_COUNT;
    joint_telemetry_msg.velocity.capacity = JOINT_COUNT;
 
    joint_telemetry_msg.effort.data = joint_efforts;
    joint_telemetry_msg.effort.size = JOINT_COUNT;
    joint_telemetry_msg.effort.capacity = JOINT_COUNT;

    // Zero initialize telemetry data arrays
    for (int i = 0; i < JOINT_COUNT; i++) {
        joint_positions[i] = 0.0;
        joint_velocities[i] = 0.0;
        joint_efforts[i] = 0.0;
    }
}



void publish_joint_telemetry(double* actual_positions, double* commanded_positions, double* commanded_speeds) {

    // Update message dataMore actions

    for (int i = 0; i < 3; i++) {

        joint_telemetry_msg.position.data[i] = actual_positions[i];     // Actual positions

        joint_telemetry_msg.effort.data[i] = commanded_positions[i];   // Commanded positions

        joint_telemetry_msg.velocity.data[i] = commanded_speeds[i];    // Commanded speeds

    }



    // Update header timestamp (optional but recommended)

    int64_t time_ms = millis();

    joint_telemetry_msg.header.stamp.sec = time_ms / 1000;

    joint_telemetry_msg.header.stamp.nanosec = (time_ms % 1000) * 1000000;



    // Publish

    RCSOFTCHECK(rcl_publish(&joint_telemetry_publisher, &joint_telemetry_msg, NULL));

}
// ----------------------
// Setup Function
// ----------------------
void setup() {
    Serial.begin(921600);
    set_microros_serial_transports(Serial);
    delay(2000);

    // Configure limit switch pins
    pinMode(LS1_NO, INPUT_PULLUP);
    pinMode(LS2_NO, INPUT_PULLUP);
    pinMode(LS3_NO, INPUT_PULLUP);
    pinMode(LS1_NC, INPUT_PULLUP);
    pinMode(LS2_NC, INPUT_PULLUP);
    pinMode(LS3_NC, INPUT_PULLUP);

    // Attach servos and set to off positions
    servo1.attach(SERVO1_PIN);
    servo2.attach(SERVO2_PIN);
    servo3.attach(SERVO3_PIN);
    servo4.attach(SERVO4_PIN);
    servo1.write(servo_off[0]);
    servo2.write(servo_off[1]);
    servo3.write(servo_off[2]);
    servo4.write(servo_off[3]);

    // Set PWM frequency for DC motor control
    float PWM_freq = 18500.0;
    analogWriteFrequency(DC1_PWM, PWM_freq);
    analogWriteFrequency(DC2_PWM, PWM_freq);
    analogWriteFrequency(DC3_PWM, PWM_freq);

    // micro-ROS allocator, support and node init
    allocator = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "psm_sensor_node", "", 
&support));

    // Subscriptions
    RCCHECK(rclc_subscription_init_default(
        &joint_state_subscriber, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
        "/mtm_joint_states"));

    RCCHECK(rclc_subscription_init_default(
        &target_pose_subscriber, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
        "/model_pose"));

    // Publishers
    RCCHECK(rclc_publisher_init_default(
        &sensor_data_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
        "/psm_sensor_data"));

    RCCHECK(rclc_publisher_init_default(
        &debug_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "/psm_debug"));

    RCCHECK(rclc_publisher_init_default(
        &joint_telemetry_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
        "/psm_joint_telemetry"));

    // Initialize sensor data message memory
    sensor_data_msg.data.data = sensor_data_array;
    sensor_data_msg.data.size = 3;
    sensor_data_msg.data.capacity = 3;

    // Initialize debug message buffer
    debug_msg.data.data = debug_data_buffer;
    debug_msg.data.size = 0;
    debug_msg.data.capacity = sizeof(debug_data_buffer);

    // Initialize telemetry message
    init_joint_telemetry_message();

    // Allocate received_joint_state buffers (JOINT_COUNT is 7)
    received_joint_state.name.data = (rosidl_runtime_c__String 
*)malloc(JOINT_COUNT * sizeof(rosidl_runtime_c__String));
    received_joint_state.name.size = JOINT_COUNT;
    received_joint_state.name.capacity = JOINT_COUNT;

    for (size_t i = 0; i < received_joint_state.name.size; i++) {
        
rosidl_runtime_c__String__init(&received_joint_state.name.data[i]);
        received_joint_state.name.data[i].data = (char *)malloc(32); // Max name length
        received_joint_state.name.data[i].capacity = 32;
        received_joint_state.name.data[i].size = 0;
    }

    received_joint_state.position.data = (double *)malloc(JOINT_COUNT * sizeof(double));
    received_joint_state.position.size = JOINT_COUNT;
    received_joint_state.position.capacity = JOINT_COUNT;

    // The following fields can be NULL if you don't intend to read them from the MTM joint states
    // but it's generally safer to allocate them if they are part of the message type
    received_joint_state.velocity.data = (double *)malloc(JOINT_COUNT * sizeof(double));
    received_joint_state.velocity.size = JOINT_COUNT;
    received_joint_state.velocity.capacity = JOINT_COUNT;
    if (received_joint_state.velocity.data) { // Check for successful allocation
        for (size_t i = 0; i < received_joint_state.velocity.size; i++) received_joint_state.velocity.data[i] = 0.0;
    }

    received_joint_state.effort.data = (double *)malloc(JOINT_COUNT * sizeof(double));
    received_joint_state.effort.size = JOINT_COUNT;
    received_joint_state.effort.capacity = JOINT_COUNT;
    if (received_joint_state.effort.data) { // Check for successful allocation
        for (size_t i = 0; i < received_joint_state.effort.size; i++) received_joint_state.effort.data[i] = 0.0;
    }


    // Allocate target_pose_msg buffers (3 for x, y, z)
    // Make sure to match the expected incoming message structure exactly.
    // The topic echo shows 'name', 'position', 'velocity', 'effort' with 3 elements each.
    target_pose_msg.name.data = (rosidl_runtime_c__String *)malloc(3 * sizeof(rosidl_runtime_c__String));
    target_pose_msg.name.size = 3;
    target_pose_msg.name.capacity = 3;
    
    const char* target_names[3] = {"x", "y", "z"}; // Populate names as they appear in the message
    for (size_t i = 0; i < target_pose_msg.name.size; i++) {
        
rosidl_runtime_c__String__init(&target_pose_msg.name.data[i]);
        target_pose_msg.name.data[i].data = (char *)malloc(32); // Max name length
        target_pose_msg.name.data[i].capacity = 32;
        strncpy(target_pose_msg.name.data[i].data, target_names[i], target_pose_msg.name.data[i].capacity - 1);
        target_pose_msg.name.data[i].data[target_pose_msg.name.data[i].capacity - 1] = '\0'; // Ensure null termination
        target_pose_msg.name.data[i].size = strlen(target_pose_msg.name.data[i].data);
    }
    

    target_pose_msg.position.data = (double *)malloc(3 * sizeof(double));
    target_pose_msg.position.size = 3;
    target_pose_msg.position.capacity = 3;    
    if (target_pose_msg.position.data) {
        for (size_t i = 0; i < target_pose_msg.position.size; i++) target_pose_msg.position.data[i] = 0.0;
    }

    // IMPORTANT: Allocate memory for velocity and effort fields for target_pose_msg
    // even if you don't directly use them in the callback, as they are part of the JointState message structure.
    target_pose_msg.velocity.data = (double *)malloc(3 * sizeof(double));
    target_pose_msg.velocity.size = 3;
    target_pose_msg.velocity.capacity = 3;
    if (target_pose_msg.velocity.data) {
        for (size_t i = 0; i < target_pose_msg.velocity.size; i++) target_pose_msg.velocity.data[i] = 0.0;
    }

    target_pose_msg.effort.data = (double *)malloc(3 * sizeof(double));
    target_pose_msg.effort.size = 3;
    target_pose_msg.effort.capacity = 3;
    if (target_pose_msg.effort.data) {
        for (size_t i = 0; i < target_pose_msg.effort.size; i++) target_pose_msg.effort.data[i] = 0.0;
    }


    // Executor for 2 subscriptions
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, 
&allocator));
    RCCHECK(rclc_executor_add_subscription(
        &executor, &joint_state_subscriber, 
&received_joint_state, &joint_state_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(
        &executor, &target_pose_subscriber, 
&target_pose_msg, &target_pose_callback, ON_NEW_DATA));

    // Publish initial debug message
    publish_debug_message("Setup Complete");
    delay(1000);

    home_motors();
    delay(2500); // Delay for ease of use
}


// ----------------------
// Loop Function
// ----------------------
void loop() {
    // 1. Read actual joint positions (roll, pitch, insertion)
    actual_positions[0] = Ax1toAngle(Enc1.read()); // Roll
    actual_positions[1] = Ax2toAngle(Enc2.read()); // Pitch
    actual_positions[2] = Ax3toAngle(Enc3.read()); // Insertion

    // 2. Get desired target angles (already implemented)
    double target_roll_angle = commanded_positions[0]; // Target roll angle
    double target_pitch_angle = commanded_positions[1]; // Target pitch angle

    // 3. Calculate gains using gain scheduling functions
    Gains roll_gains = getRollGains(actual_positions[0], actual_positions[1]); // Roll gains
    Gains pitch_gains = getPitchGains(actual_positions[1], actual_positions[0]); // Pitch gains

    // 4. Command joints using LQI controller
    // Roll axis control
    float roll_lqi_gains[2] = {static_cast<float>(roll_gains.Kp), static_cast<float>(roll_gains.Kd)};
    float roll_speed = compute_roll_LQR_control(
        roll_lqi_gains,       // Gains for roll
        target_roll_angle,    // Target roll angle
        actual_positions[0]   // Actual roll position
    );
    motor1.setSpeed(static_cast<int16_t>(roll_speed)); // Apply roll control
    commanded_speeds[0] = roll_speed; // Store commanded roll speed

    // Pitch axis control
    float pitch_lqi_gains[3] = {static_cast<float>(pitch_gains.Kp), 
                                static_cast<float>(pitch_gains.Kd), 
                                static_cast<float>(pitch_gains.Ki)};
    float pitch_speed = compute_pitch_LQI_control(
        pitch_lqi_gains,      // Gains for pitch
        target_pitch_angle,   // Target pitch angle
        actual_positions[1]   // Actual pitch position
    );
    motor2.setSpeed(static_cast<int16_t>(pitch_speed)); // Apply pitch control
    commanded_speeds[1] = pitch_speed; // Store commanded pitch speed

    // 5. Read raw sensor data
    read_encoder_data(&sensor_data_msg);
    RCSOFTCHECK(rcl_publish(&sensor_data_publisher, &sensor_data_msg, NULL));

    // 6. Process incoming ROS messages
    RCSOFTCHECK(rclc_executor_spin_some(&executor, 10));

    // 7. Publish telemetry
    publish_joint_telemetry(actual_positions, commanded_positions, commanded_speeds);

    // 8. Maintain loop timing
    delay(2); // Sampling rate of 500 Hz (2 ms per iteration)
}


// PRBS Characterization DO NOT DELETE
// void loop() {
//     static unsigned long start_time = millis(); // Record the start time
//     static bool prbs_started = false;          // Flag to indicate PRBS has started
//     static unsigned long prbs_last_toggle_time = 0; // Last time PRBS toggled
//     static float prbs_value = 0.0f;            // Current PRBS value
//     float roll_target_position = 0.0f; // Target position for roll axis
//     float pitch_target_position = -15.0f; // Target position for pitch axis

//     // PRBS parameters
//     const float prbs_amplitude = 15.0f; // Amplitude of PRBS
//     const unsigned long prbs_intervals[] = {5, 10, 20, 30, 50, 70, 100, 250, 500, 1000}; // PRBS toggle intervals (ms)
//     const int num_intervals = sizeof(prbs_intervals) / sizeof(prbs_intervals[0]);
//     static int current_interval_index = 0; // Current interval index

//     unsigned long current_time = millis();
//     float elapsed_time = (current_time - start_time) / 1000.0f; // Elapsed time in seconds

//     // 1. Read filtered encoder values
//     actual_positions[0] = Ax1toAngle(Enc1.read()); // Roll
//     actual_positions[1] = Ax2toAngle(Enc2.read()); // Pitch
//     actual_positions[2] = Ax3toAngle(Enc3.read()); // Insertion

//     // == Roll Axis Adaptive Control ==
//     Gains roll_gains = getRollGains(actual_positions[0], actual_positions[1]); // Get adaptive gains
//     float roll_lqr_gains[2] = {static_cast<float>(roll_gains.Kp), static_cast<float>(roll_gains.Kd)};
//     float roll_speed = compute_roll_LQR_control(
//         roll_lqr_gains, 
//         roll_target_position,               // Target position for roll is 0 degrees
//         actual_positions[0] // Actual position (roll)
//     );
//     motor1.setSpeed(static_cast<int16_t>(roll_speed)); // Apply LQR control to motor1
//     commanded_speeds[0] = roll_speed; // Store commanded speed for telemetry

//     // == Pitch Axis Control ==
//     if (elapsed_time < 15.0f) {
//         // Light LQR control for the first 15 seconds
//         float pitch_lqr_gains[2] = {30.4189f, 0.390f}; // Light LQR gains
//         float pitch_speed = compute_pitch_LQR_control(
//             pitch_lqr_gains, 
//             pitch_target_position,            // Target position for pitch is -15 degrees
//             actual_positions[1] // Actual position (pitch)
//         );
//         motor2.setSpeed(static_cast<int16_t>(pitch_speed)); // Apply LQR control to motor2
//         commanded_speeds[1] = pitch_speed; // Store commanded speed for telemetry
//     } else {
//         // After 15 seconds, apply PRBS signals
//         if (!prbs_started) {
//             prbs_started = true;
//             prbs_last_toggle_time = current_time; // Initialize PRBS toggle time
//         }

//         // Check if it's time to toggle PRBS
//         if ((current_time - prbs_last_toggle_time) >= prbs_intervals[current_interval_index]) {
//             // Toggle PRBS value between +amplitude and -amplitude
//             prbs_value = (random(0, 2) == 0) ? prbs_amplitude : -prbs_amplitude;
//             prbs_last_toggle_time = current_time; // Update last toggle time

//             // Cycle to the next interval
//             current_interval_index = (current_interval_index + 1) % num_intervals;
//         }

//         // LQR + PRBS control
//         float pitch_lqr_gains[2] = {30.4189f, 0.390f}; // Light LQR gains
//         float pitch_speed = compute_pitch_LQR_control(
//             pitch_lqr_gains, 
//             pitch_target_position,            // Target position for pitch is -15 degrees
//             actual_positions[1] // Actual position (pitch)
//         );

//         float prbs_control_input = pitch_speed + prbs_value; // Combine LQR control with PRBS
//         motor2.setSpeed(static_cast<int16_t>(prbs_control_input)); // Apply control to motor2
//         commanded_speeds[1] = prbs_control_input; // Store commanded speed for telemetry
//     }

//     // 2. Read raw sensor data
//     read_encoder_data(&sensor_data_msg);
//     RCSOFTCHECK(rcl_publish(&sensor_data_publisher, &sensor_data_msg, NULL));

//     // 3. Process incoming ROS messages
//     RCSOFTCHECK(rclc_executor_spin_some(&executor, 10));

//     // 4. Publish telemetry
//     publish_joint_telemetry(actual_positions, commanded_positions, commanded_speeds);

//     // 5. Maintain loop timing
//     delay(2); // Sampling rate of 500 Hz (2 ms per iteration)
// }
// Control algorithm testing, do not delete
// void loop() {
//     static unsigned long start_time = millis(); // Record the start time

//     // 1. Read filtered encoder values
//     actual_positions[0] = Ax1toAngle(Enc1.read()); // Roll
//     actual_positions[1] = Ax2toAngle(Enc2.read()); // Pitch
//     actual_positions[2] = Ax3toAngle(Enc3.read()); // Insertion

//     unsigned long current_time = millis();
//     float elapsed_time = (current_time - start_time) / 1000.0f; // Elapsed time in seconds

//     // == Calculate Target Positions ==
//     const float pitch_amplitude = 15.0f; // Amplitude for pitch sine wave
//     const float roll_amplitude = 20.0f;  // Amplitude for roll sine wave
//     const float pitch_period = 10.0f;     // Period for pitch sine wave (seconds)
//     const float roll_period = 11.0f;      // Period for roll sine wave (seconds)

//     // Calculate target positions using sine wave equations
//     float target_pitch_angle = pitch_amplitude * sin((2.0f * PI / pitch_period) * elapsed_time);
//     float target_roll_angle = roll_amplitude * sin((2.0f * PI / roll_period) * elapsed_time);
//     commanded_positions[0] = target_roll_angle; // Store commanded roll position
//     commanded_positions[1] = target_pitch_angle; // Store commanded pitch position

//     // == Roll Axis Control ==
//     // Get roll gains based on the current roll and pitch angles
//     Gains roll_gains = getRollGains(actual_positions[0], actual_positions[1]);

//     // Publish the roll gains as a debug message
//     publish_gains(roll_gains.Kp, roll_gains.Kd);

//     // Create a temporary array for the gains
//     float roll_lqr_gains[2] = {static_cast<float>(roll_gains.Kp), static_cast<float>(roll_gains.Kd)};

//     // Use the calculated gains for LQR control
//     float roll_speed = compute_roll_LQR_control(
//         roll_lqr_gains,       // Pass the array of gains
//         target_roll_angle,    // Target position for roll
//         actual_positions[0]   // Actual position (roll)
//     );
//     motor1.setSpeed(static_cast<int16_t>(roll_speed)); // Apply LQR control to motor1
//     commanded_speeds[0] = roll_speed; // Store commanded speed for telemetry

//     // == Pitch Axis Control ==
//     // Get pitch gains based on the current pitch and roll angles
//     Gains pitch_gains = getPitchGains(actual_positions[1], actual_positions[0]);

//     // Publish the pitch gains as a debug message
//     char debug_message[128];
//     snprintf(debug_message, sizeof(debug_message), "Pitch Gains - Kp: %.2f, Kd: %.2f, Ki: %.2f", 
//              pitch_gains.Kp, pitch_gains.Kd, pitch_gains.Ki);
//     debug_msg.data.size = strlen(debug_message);
//     strncpy(debug_msg.data.data, debug_message, debug_msg.data.capacity);
//     RCSOFTCHECK(rcl_publish(&debug_publisher, &debug_msg, NULL));

//     // Create a temporary array for the gains
//     float pitch_lqi_gains[3] = {static_cast<float>(pitch_gains.Kp), 
//                                 static_cast<float>(pitch_gains.Kd), 
//                                 static_cast<float>(pitch_gains.Ki)};

//     // Use the calculated gains for LQI control
//     float pitch_speed = compute_pitch_LQI_control(
//         pitch_lqi_gains, 
//         target_pitch_angle, // Target position for pitch
//         actual_positions[1] // Actual position (pitch)
//     );
//     motor2.setSpeed(static_cast<int16_t>(pitch_speed)); // Apply LQI control to motor2
//     commanded_speeds[1] = pitch_speed; // Store commanded speed for telemetry

//     // 2. Read raw sensor data
//     read_encoder_data(&sensor_data_msg);
//     RCSOFTCHECK(rcl_publish(&sensor_data_publisher, &sensor_data_msg, NULL));

//     // 3. Process incoming ROS messages
//     RCSOFTCHECK(rclc_executor_spin_some(&executor, 10));

//     // 4. Publish telemetry
//     publish_joint_telemetry(actual_positions, commanded_positions, commanded_speeds);

//     // 5. Maintain loop timing
//     delay(2); // Sampling rate of 500 Hz (2 ms per iteration)
// }

// Roll step response script Do not delete
// void loop() {
//     static unsigned long start_time = millis(); // Record the start time
//     static unsigned long phase_start_time = millis(); // Track the start time of each phase
//     static float last_roll_speed = 0.0f; // Store the last commanded roll speed
//     static int phase = 0; // Track the current phase (0: LQR, 1: Hold speed, 2: Double speed)

//     // 1. Read filtered encoder values
//     actual_positions[0] = Ax1toAngle(Enc1.read()); // Roll
//     actual_positions[1] = Ax2toAngle(Enc2.read()); // Pitch
//     actual_positions[2] = Ax3toAngle(Enc3.read()); // Insertion

//     unsigned long current_time = millis();
//     unsigned long elapsed_time = current_time - phase_start_time;

//     // Phase 0: LQR control for the first 10 seconds
//     if (phase == 0) {
//         if (elapsed_time < 10000) { // 10 seconds
//             float target_roll_angle = -15.0f; // Command the roll axis to -15 degrees
//             float roll_lqr_gains[2] = {200.1886f, 3.0860f}; // Gains for position error and velocity
//             float roll_speed = compute_roll_LQR_control(
//                 roll_lqr_gains, 
//                 target_roll_angle, // Target position for roll
//                 actual_positions[0] // Actual position (roll)
//             );
//             motor1.setSpeed(static_cast<int16_t>(roll_speed)); // Apply LQR control to motor1
//             commanded_speeds[0] = roll_speed; // Store commanded speed for telemetry

//             // Store the last commanded roll speed
//             last_roll_speed = roll_speed;
//         } else {
//             // Transition to Phase 1
//             phase = 1;
//             phase_start_time = millis(); // Reset phase start time
//         }
//     }

//     // Phase 1: Hold the last commanded speed for 5 seconds
//     else if (phase == 1) {
//         if (elapsed_time < 5000) { // 5 seconds
//             motor1.setSpeed(static_cast<int16_t>(last_roll_speed)); // Maintain the last commanded speed
//             commanded_speeds[0] = last_roll_speed; // Store commanded speed for telemetry
//         } else {
//             // Transition to Phase 2
//             phase = 2;
//             phase_start_time = millis(); // Reset phase start time
//         }
//     }

//     // Phase 2: Double the last commanded speed
//     else if (phase == 2) {
//         float doubled_speed = 1.9f * last_roll_speed; // Double the last commanded speed
//         motor1.setSpeed(static_cast<int16_t>(doubled_speed)); // Apply the doubled speed to motor1
//         commanded_speeds[0] = doubled_speed; // Store commanded speed for telemetry

//         // No further transitions; this phase continues indefinitely
//     }

//     // == Command the Pitch Axis ==
//     float target_pitch_angle = 15.0f; // Command the pitch axis to -15 degrees
//     float pitch_lqi_gains[3] = {220.6809f, 0.3174f, 0.1414f}; // Gains for position error, velocity, and integral
//     float pitch_speed = compute_pitch_LQI_control(
//         pitch_lqi_gains, 
//         target_pitch_angle, // Target position for pitch
//         actual_positions[1] // Actual position (pitch)
//     );
//     motor2.setSpeed(static_cast<int16_t>(pitch_speed)); // Apply LQI control to motor2
//     commanded_speeds[1] = pitch_speed; // Store commanded speed for telemetry

//     // 2. Read raw sensor data
//     read_encoder_data(&sensor_data_msg);
//     RCSOFTCHECK(rcl_publish(&sensor_data_publisher, &sensor_data_msg, NULL));

//     // 3. Process incoming ROS messages
//     RCSOFTCHECK(rclc_executor_spin_some(&executor, 10));

//     // 4. Publish telemetry
//     publish_joint_telemetry(actual_positions, commanded_positions, commanded_speeds);

//     // 5. Maintain loop timing
//     delay(2); // Sampling rate of 500 Hz (2 ms per iteration)
// }


/* PRBS Characterization. Do not delete
void loop() {
    static unsigned long start_time = millis();
    static bool prbs_started = false;2
    static float prbs_value = 0.0f;

    // PRBS parameters
    const float prbs_amplitude = 15.0f;

    // Array of toggle intervals in milliseconds
    const unsigned long prbs_intervals[] = {5, 10, 20, 30, 50, 70, 100, 250, 500, 1000};
    const int num_intervals = sizeof(prbs_intervals) / sizeof(prbs_intervals[0]);
    static int current_interval_index = 0;

    unsigned long current_time = millis();
    float elapsed_time = (current_time - start_time) / 1000.0f;

    // 1. Read filtered encoder values
    actual_positions[0] = Ax1toAngle(Enc1.read());
    actual_positions[1] = Ax2toAngle(Enc2.read());
    actual_positions[2] = Ax3toAngle(Enc3.read());

    // == Roll Axis LQR ==
    float roll_lqr_gains[2] = {255.1886f, 4.0860f};
    float roll_speed = compute_roll_LQR_control(
        roll_lqr_gains, 
        0.0f,
        actual_positions[0]
    );
    motor1.setSpeed(static_cast<int16_t>(roll_speed));
    commanded_speeds[0] = roll_speed;

    // == Pitch Axis ==
    if (elapsed_time < 15.0f) {
        float pitch_lqr_gains[2] = {50.4189f, 0.590f};
        float pitch_speed = compute_pitch_LQR_control(
            pitch_lqr_gains, 
            0.0f,
            actual_positions[1]
        );
        motor2.setSpeed(static_cast<int16_t>(pitch_speed));
        commanded_speeds[1] = pitch_speed;
    } else {
        if (!prbs_started) {
            prbs_started = true;
            prbs_last_toggle_time = current_time;
        }

        // Check for PRBS toggle
        if ((current_time - prbs_last_toggle_time) >= prbs_intervals[current_interval_index]) {
            prbs_value = (random(0, 2) == 0) ? prbs_amplitude : -prbs_amplitude;
            prbs_last_toggle_time = current_time;

            // Cycle to next interval value
            current_interval_index = (current_interval_index + 1) % num_intervals;
        }

        // LQR + PRBS control
        float pitch_lqr_gains[2] = {50.4189f, 0.590f};
        float pitch_speed = compute_pitch_LQR_control(
            pitch_lqr_gains, 
            0.0f,
            actual_positions[1]
        );

        float prbs_control_input = pitch_speed + prbs_value;

        motor2.setSpeed(static_cast<int16_t>(prbs_control_input));
        commanded_speeds[1] = prbs_control_input;
    }

    // 2. Read raw sensor data
    read_encoder_data(&sensor_data_msg);
    RCSOFTCHECK(rcl_publish(&sensor_data_publisher, &sensor_data_msg, NULL));

    // 3. Process incoming ROS messages
    RCSOFTCHECK(rclc_executor_spin_some(&executor, 10));

    // 4. Publish telemetry
    publish_joint_telemetry(actual_positions, commanded_positions, commanded_speeds);

    // 5. Maintain loop timing
    delay(2);
}
 */

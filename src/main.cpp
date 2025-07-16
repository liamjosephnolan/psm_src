#include "config.h" // All necessary libraries and macros are included here

// ----------------------
// Global ROS 2 Objects
// ----------------------
rcl_subscription_t joint_state_subscriber;
rcl_subscription_t target_pose_subscriber;
rcl_publisher_t sensor_data_publisher;
rcl_publisher_t debug_publisher;
rcl_publisher_t joint_telemetry_publisher;  // Telemetry publisher
rcl_publisher_t gains_publisher; // Assuming this is declared but not initialized/used yet

// Declare global message objects
sensor_msgs__msg__JointState received_joint_state;
sensor_msgs__msg__JointState target_pose_msg; // This is the one for /model_pose
std_msgs__msg__Int32MultiArray sensor_data_msg;
std_msgs__msg__String debug_msg;
sensor_msgs__msg__JointState joint_telemetry_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// --- Control Mode Variable ---
// Valid options: "target" (for ROS control) or "sin" (for sine wave control) or "traj" (for trajectory control)
String control_mode = "traj"; // <--- SET YOUR DESIRED MODE HERE PERMANENTLY
unsigned long sine_wave_start_time_ms = 0; // To ensure smooth sine wave start

// --- Trajectory Control State Machine ---
// This enum defines the different states the trajectory mode can be in.
enum TrajectoryState {
    INITIAL_HOLD, // The initial phase, holding the first position.
    EXECUTING,    // Actively following the trajectory points.
    FINISHED      // The final phase, holding the last position.
};
TrajectoryState traj_state = INITIAL_HOLD; // Start in the initial hold state.

// Trajectory variables
std::vector<TrajPoint> trajectory; // Holds the parsed trajectory data.
int traj_index = 0; // The current index in the trajectory vector.
unsigned long traj_start_time = 0; // Timestamp for when trajectory execution begins (after the hold).
unsigned long initial_hold_start_time = 0; // Timestamp for when the initial hold period starts.


// ----------------------
// Servo and Motor Control (instantiation of externs from config.h)
// ----------------------
Servo servo1, servo2, servo3, servo4; // Servo objects

// Instantiate CytronMD motors directly using the enum from the library
CytronMD motor1(PWM_DIR, DC1_PWM, DC1_DIR); // This uses the PWM_DIR enum member from CytronMotorDriver.h
CytronMD motor2(PWM_DIR, DC2_PWM, DC2_DIR);
CytronMD motor3(PWM_DIR, DC3_PWM, DC3_DIR);
CytronMD motor[3] = {motor1, motor2, motor3}; // Array of motors

// Servo Calibration Arrays
int servo_off[4] = {100, 97, 90, 92};
int servo_val[4] = {0, 0, 0, 0};

// Initialize telemetry message variables (instantiation of externs from config.h)
double actual_positions[3] = {0.0, 0.0, 0.0};
double commanded_positions[3] = {0.0, 0.0, 0.0};
double commanded_speeds[3] = {0.0, 0.0, 0.0};

// Voltage variables for motors (instantiation of extern from config.h)
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
// Static Buffers for *Received* Messages
// ----------------------
// For received_joint_state (from /mtm_joint_states, JOINT_COUNT joints)
rosidl_runtime_c__String received_joint_state_names_array[JOINT_COUNT];
char received_joint_state_names_data[JOINT_COUNT][32]; // Max name length for each string
double received_joint_state_positions_array[JOINT_COUNT];
double received_joint_state_velocities_array[JOINT_COUNT];
double received_joint_state_efforts_array[JOINT_COUNT];

// For target_pose_msg (from /model_pose, 3 elements for x, y, z)
rosidl_runtime_c__String target_pose_msg_names_array[3];
char target_pose_msg_names_data[3][32]; // Max name length for each string
double target_pose_msg_positions_array[3];
double target_pose_msg_velocities_array[3];
double target_pose_msg_efforts_array[3];


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

    const sensor_msgs__msg__JointState *msg = (const sensor_msgs__msg__JointState *)msgin;

    // Check if the received message has enough data.
    // The sizes are populated by micro-ROS upon reception.
    if (msg->position.size < 7 || msg->position.data == NULL) {
        publish_debug_message("[WARN] Invalid joint state message.");
        return;
    }

    // Extract joint positions
    // These indices (6, 5, 4, 3) map to specific joints based on the MTM's JointState message structure
    float g0 = msg->position.data[6]; // Grasper pinch
    float g1 = msg->position.data[5]; // Grasper tilt
    float g2 = msg->position.data[4]; // Grasper pitch
    float g3 = msg->position.data[3]; // Grasper roll

    // Calculate disk movements for servos based on grasper joint commands
    float disk_movements[4];
    calculate_disk_movements(g3, g2, g1, g0, disk_movements);

    // Account for servo offsets
    disk_movements[0] += servo_off[3];
    disk_movements[1] += servo_off[2];
    disk_movements[2] += servo_off[0];
    disk_movements[3] += servo_off[1];

    // Command servos
    servo1.write(static_cast<int>(disk_movements[2]));
    servo2.write(static_cast<int>(disk_movements[3]));
    servo3.write(static_cast<int>(disk_movements[1]));
    servo4.write(static_cast<int>(disk_movements[0]));

    // Store commanded servo values in telemetry effort field
    // Ensure JOINT_COUNT is large enough (at least 7) to avoid out-of-bounds access
    if (JOINT_COUNT >= 7) {
        joint_telemetry_msg.effort.data[3] = disk_movements[2];
        joint_telemetry_msg.effort.data[4] = disk_movements[3];
        joint_telemetry_msg.effort.data[5] = disk_movements[1];
        joint_telemetry_msg.effort.data[6] = disk_movements[0];
    } else {
        publish_debug_message("[ERROR] JOINT_COUNT too small for servo telemetry!");
    }
}

// Target pose callback (now receiving a JointState message with positions x, y, z)
void target_pose_callback(const void *msgin) {

    const sensor_msgs__msg__JointState *msg = (const sensor_msgs__msg__JointState *)msgin;

    // Check if the received message has enough data.
    // The sizes are populated by micro-ROS upon reception.
    if (msg->position.size < 3 || msg->position.data == NULL) {
        publish_debug_message("[WARN] Invalid target pose message.");
        return;
    }

    // Compute PSM joint angles from the received x, y, z positions
    JointAngles angles = computePSMJointAngles(msg->position.data[0], msg->position.data[1], msg->position.data[2]);

    // Only update commanded_positions from ROS if control_mode is "target"
    if (control_mode == "target") {
        commanded_positions[0] = angles.q1; // Target value for roll equivalent
        commanded_positions[1] = angles.q2; // Target value for pitch equivalent
        commanded_positions[2] = angles.q3; // Target value for insertion equivalent
    }

    // Store computed angles in telemetry position fields
    if (JOINT_COUNT >= 3) {
        joint_telemetry_msg.position.data[0] = angles.q1; // Roll (computed from IK)
        joint_telemetry_msg.position.data[1] = angles.q2; // Pitch (computed from IK)
        joint_telemetry_msg.position.data[2] = angles.q3; // Insertion (computed from IK)
    } else {
        publish_debug_message("[ERROR] JOINT_COUNT too small for angle telemetry!");
    }
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
        joint_names[i].size = strlen(names[i]); // Set initial size
        joint_names[i].capacity = sizeof(joint_names_data[i]);
        strncpy(joint_names[i].data, names[i], joint_names[i].capacity - 1); // Copy name
        joint_names[i].data[joint_names[i].capacity - 1] = '\0'; // Ensure null termination
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

void publish_joint_telemetry(double* actual_positions_arr, double* commanded_positions_arr, double* commanded_speeds_arr) {
    // Update message data for the arm joints (Roll, Pitch, Insertion)
    // Note: Parameter names changed to avoid shadowing global variables.
    for (int i = 0; i < 3; i++) {
        joint_telemetry_msg.position.data[i] = actual_positions_arr[i];     // Actual positions (roll, pitch, insertion)
        joint_telemetry_msg.effort.data[i] = commanded_positions_arr[i];   // Commanded positions (from ROS topic or sine wave)
        joint_telemetry_msg.velocity.data[i] = commanded_speeds_arr[i];    // Commanded speeds (roll, pitch, insertion speeds)
    }
    // Servo data (indices 3-6) are updated in joint_state_callback.

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
    // Serial.begin(921600); // Initialize Serial for debug and commands - REMOVED
    set_microros_serial_transports(Serial); // Keep this if micro-ROS uses Serial for transport
    delay(2000); // Give time for micro-ROS to connect/board to stabilize

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
    float PWM_freq = 18500.0; // Typically high frequency for smoother DC motor control
    analogWriteFrequency(DC1_PWM, PWM_freq);
    analogWriteFrequency(DC2_PWM, PWM_freq);
    analogWriteFrequency(DC3_PWM, PWM_freq);

    // micro-ROS allocator, support and node init
    allocator = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "psm_sensor_node", "", &support));

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
    debug_msg.data.capacity = sizeof(debug_data_buffer); // FIXED: Changed debug_msg.capacity to debug_msg.data.capacity

    // Initialize telemetry message
    init_joint_telemetry_message();

    // *******************************************************************
    // CRITICAL CHANGE: Use static buffers for received messages
    // instead of malloc. micro-ROS manages filling these.
    // *******************************************************************

    // Initialize received_joint_state buffers (JOINT_COUNT is 7)
    received_joint_state.name.data = received_joint_state_names_array;
    received_joint_state.name.size = JOINT_COUNT; // Initial size, will be updated by rclc
    received_joint_state.name.capacity = JOINT_COUNT;
    for (size_t i = 0; i < JOINT_COUNT; i++) {
        rosidl_runtime_c__String__init(&received_joint_state.name.data[i]);
        received_joint_state.name.data[i].data = received_joint_state_names_data[i];
        received_joint_state.name.data[i].capacity = sizeof(received_joint_state_names_data[i]);
        received_joint_state.name.data[i].size = 0; // Initial size, will be updated by rclc
    }

    received_joint_state.position.data = received_joint_state_positions_array;
    received_joint_state.position.size = JOINT_COUNT; // Initial size, will be updated by rclc
    received_joint_state.position.capacity = JOINT_COUNT;

    received_joint_state.velocity.data = received_joint_state_velocities_array;
    received_joint_state.velocity.size = JOINT_COUNT; // Initial size, will be updated by rclc
    received_joint_state.velocity.capacity = JOINT_COUNT;

    received_joint_state.effort.data = received_joint_state_efforts_array;
    received_joint_state.effort.size = JOINT_COUNT; // Initial size, will be updated by rclc
    received_joint_state.effort.capacity = JOINT_COUNT;


    // Initialize target_pose_msg buffers (3 for x, y, z)
    target_pose_msg.name.data = target_pose_msg_names_array;
    target_pose_msg.name.size = 3; // Initial size, will be updated by rclc
    target_pose_msg.name.capacity = 3;
    const char* target_names[3] = {"x", "y", "z"}; // Populate names as they appear in the message
    for (size_t i = 0; i < target_pose_msg.name.size; i++) {
        rosidl_runtime_c__String__init(&target_pose_msg.name.data[i]);
        target_pose_msg.name.data[i].data = target_pose_msg_names_data[i];
        target_pose_msg.name.data[i].capacity = sizeof(target_pose_msg_names_data[i]);
        strncpy(target_pose_msg.name.data[i].data, target_names[i], target_pose_msg.name.data[i].capacity - 1);
        target_pose_msg.name.data[i].data[target_pose_msg.name.data[i].capacity - 1] = '\0'; // Ensure null termination
        target_pose_msg.name.data[i].size = strlen(target_names[i]); // Set initial size based on copied string
    }


    target_pose_msg.position.data = target_pose_msg_positions_array;
    target_pose_msg.position.size = 3; // Initial size, will be updated by rclc
    target_pose_msg.position.capacity = 3;

    target_pose_msg.velocity.data = target_pose_msg_velocities_array;
    target_pose_msg.velocity.size = 3; // Initial size, will be updated by rclc
    target_pose_msg.velocity.capacity = 3;

    target_pose_msg.effort.data = target_pose_msg_efforts_array;
    target_pose_msg.effort.size = 3; // Initial size, will be updated by rclc
    target_pose_msg.effort.capacity = 3;


    // Executor for 2 subscriptions
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
    RCCHECK(rclc_executor_add_subscription(
        &executor, &joint_state_subscriber,
        &received_joint_state, &joint_state_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(
        &executor, &target_pose_subscriber,
        &target_pose_msg, &target_pose_callback, ON_NEW_DATA));

    // Publish initial debug message
    publish_debug_message(("Setup Complete. Control mode set to " + control_mode + ".").c_str()); // FIXED: .c_str() added
    delay(1000);

    home_motors();
    delay(2500); // Delay for ease of use

    // Set sine wave start time when setup completes, so it always starts from 0 when Arduino boots
    // Only relevant if control_mode is "sin"
    if (control_mode == "sin") {
        sine_wave_start_time_ms = millis();
    }

    if (control_mode == "traj") {
        parse_traj_file();
        initial_hold_start_time = millis(); // Start the 10-second hold timer.
        char debug_msg_buffer[128];
        snprintf(debug_msg_buffer, sizeof(debug_msg_buffer), "Trajectory mode initialized. Trajectory size: %d. Starting initial hold.", trajectory.size());
        publish_debug_message(debug_msg_buffer);
    }
}

// ----------------------
// Loop Function
// ----------------------
void loop() {
    // 1. Read actual joint positions (roll, pitch, insertion)
    actual_positions[0] = Ax1toAngle(Enc1.read()); // Roll
    actual_positions[1] = Ax2toAngle(Enc2.read()); // Pitch
    actual_positions[2] = Ax3toAngle(Enc3.read()); // Insertion

    // 2. Determine desired target angles/positions based on control_mode
    double target_roll_angle = 0.0;
    double target_pitch_angle = 0.0;
    double target_insertion_position = 0.0;

    if (control_mode == "sin") {
        float time_s = (millis() - sine_wave_start_time_ms) / 1000.0f; // Time in seconds, make sure this is float arithmetic

        // Calculate sine wave targets for each DOF using the #defined constants
        target_roll_angle = ROLL_SINE_AMPLITUDE * sin(2.0f * M_PI * time_s / ROLL_SINE_PERIOD);
        // target_roll_angle = 0.0f; // Roll is not used in sine wave control, set to 0
        // target_pitch_angle = PITCH_SINE_AMPLITUDE * sin(2.0f * M_PI * time_s / PITCH_SINE_PERIOD);
        target_pitch_angle = 0.0f; // Pitch is not used in sine wave control, set to 0
        // target_insertion_position = INSERTION_SINE_AMPLITUDE * sin(2.0f * M_PI * time_s / INSERTION_SINE_PERIOD);
    } else if (control_mode == "traj") {
        if (!trajectory.empty()) { // Ensure the trajectory is not empty before proceeding.
            switch (traj_state) {
                case INITIAL_HOLD: {
                    // Command the system to the first point of the trajectory.

                    JointAngles angles = computePSMJointAngles(trajectory[0].x,trajectory[0].y, trajectory[0].z);
                    target_roll_angle = angles.q1; // Roll equivalent
                    target_pitch_angle = angles.q2; // Pitch equivalent
                    target_insertion_position = angles.q3; // Insertion equivalent
                    
                    {
                        char debug_msg_buffer[128];
                        snprintf(debug_msg_buffer, sizeof(debug_msg_buffer), "INITIAL_HOLD: Commanding to (%.4f, %.4f, %.4f)", angles.q1,angles.q2, angles.q3);
                        publish_debug_message(debug_msg_buffer);
                    }

                    // Check if the 10-second hold time has elapsed.
                    if (millis() - initial_hold_start_time > 10000) {
                        traj_state = EXECUTING; // Transition to the executing state.
                        traj_start_time = millis(); // Record the start time for trajectory execution.
                        traj_index = 0; // Reset index to start from the beginning of the trajectory.
                        publish_debug_message("Transitioning from INITIAL_HOLD to EXECUTING.");
                    }
                    break;
                }

                case EXECUTING: {
                    double elapsed_time = (millis() - traj_start_time) / 1000.0;

                    // Find the current segment of the trajectory
                    // traj_index should point to the *start* of the current segment
                    while (static_cast<size_t>(traj_index) < trajectory.size() - 1 && elapsed_time >= trajectory[traj_index + 1].time) {
                        traj_index++;
                    }

                    if (static_cast<size_t>(traj_index) < trajectory.size()) {
                        TrajPoint p0 = trajectory[traj_index];
                        
                        // If we are at the last point or beyond, just command the last point
                        if (static_cast<size_t>(traj_index) == trajectory.size() - 1 || elapsed_time >= p0.time) {
                            JointAngles angles = computePSMJointAngles(p0.x,p0.y, p0.z);
                            target_roll_angle = angles.q1; // Roll equivalent
                            target_pitch_angle = angles.q2; // Pitch equivalent
                            target_insertion_position = angles.q3; // Insertion equivalent
                            
                            // If we've reached the end of the trajectory, transition to FINISHED
                            if (static_cast<size_t>(traj_index) == trajectory.size() - 1 && elapsed_time >= p0.time) {
                                traj_state = FINISHED;
                                publish_debug_message("Transitioning from EXECUTING to FINISHED.");
                            }
                        } else {
                            // Interpolate between the current point (p0) and the next point (p1)
                            TrajPoint p1 = trajectory[traj_index + 1];
                            double segment_duration = p1.time - p0.time;
                            double time_in_segment = elapsed_time - p0.time;
                            double ratio = (segment_duration > 0) ? (time_in_segment / segment_duration) : 0.0;

                            target_roll_angle = p0.x + ratio * (p1.x - p0.x);
                            target_pitch_angle = p0.y + ratio * (p1.y - p0.y);
                            target_insertion_position = p0.z + ratio * (p1.z - p0.z);
                            JointAngles angles = computePSMJointAngles(target_roll_angle,target_pitch_angle, target_insertion_position);
                            target_roll_angle = angles.q1; // Roll equivalent
                            target_pitch_angle = angles.q2; // Pitch equivalent
                            target_insertion_position = angles.q3; // Insertion equivalent
                        }

                        commanded_positions[0] = target_roll_angle;
                        commanded_positions[1] = target_pitch_angle;
                        commanded_positions[2] = target_insertion_position;

                        char debug_msg_buffer[128];
                        snprintf(debug_msg_buffer, sizeof(debug_msg_buffer), "EXECUTING: t_elapsed=%.2f, traj_idx=%d, x=%.4f, y=%.4f, z=%.4f", elapsed_time, traj_index, target_roll_angle, target_pitch_angle, target_insertion_position);
                        publish_debug_message(debug_msg_buffer);

                    } else {
                        // This case should ideally be caught by the while loop and transition to FINISHED
                        // but as a fallback, if somehow traj_index goes out of bounds, transition to FINISHED.
                        traj_state = FINISHED;
                        publish_debug_message("Transitioning from EXECUTING to FINISHED (fallback).");
                    }
                    break;
                }

                case FINISHED: {
                    // Hold the last commanded position indefinitely.
                    target_roll_angle = commanded_positions[0];
                    target_pitch_angle = commanded_positions[1];
                    target_insertion_position = commanded_positions[2];
                    publish_debug_message("FINISHED: Holding last position.");
                    break;
                }
            }
        }
    } else { // control_mode == "target"
        // Use values received from ROS /model_pose topic (updated in target_pose_callback)
        publish_debug_message("TARGET MODE ENTER");
        target_roll_angle = commanded_positions[0];
        target_pitch_angle = commanded_positions[1];
        target_insertion_position = commanded_positions[2];
    }

    // 3. Calculate gains using gain scheduling functions
    Gains pitch_gains = getPitchGains(actual_positions[1], actual_positions[0]); // Pitch gains
    float pitch_lqi_gains[3] = {static_cast<float>(pitch_gains.Kp),
        static_cast<float>(pitch_gains.Kd),
        static_cast<float>(pitch_gains.Ki)};
    Gains roll_gains = getRollGains(actual_positions[1], actual_positions[0]);
    float roll_lqr_gains[2] = {static_cast<float>(roll_gains.Kp),
    static_cast<float>(roll_gains.Kd)};
    

    // 4. Command joints using LQI controller

    // Roll axis control
    float roll_speed = compute_roll_LQR_control(
        roll_lqr_gains,       // Gains for roll
        static_cast<float>(target_roll_angle),    // Target roll angle (from chosen mode)
        static_cast<float>(actual_positions[0])   // Actual roll position
    );
    motor1.setSpeed(static_cast<int16_t>(roll_speed)); // Apply roll control
    commanded_speeds[0] = roll_speed; // Store commanded roll speed for telemetry

    // Pitch axis control
    float pitch_speed = compute_pitch_LQI_control(
        pitch_lqi_gains,      // Gains for pitch
        static_cast<float>(target_pitch_angle),   // Target pitch angle (from chosen mode)
        static_cast<float>(actual_positions[1])   // Actual pitch position
    );

    motor2.setSpeed(static_cast<int16_t>(pitch_speed)); // Apply pitch control
    commanded_speeds[1] = pitch_speed; // Store commanded pitch speed for telemetry

    // Insertion axis control (motor3)
        float insertion_error = static_cast<float>(target_insertion_position - actual_positions[2]);
        float insertion_speed = 30.0f * insertion_error; // Calculate speed based on error (P-control like)

        // Clamp the speed to the range [-100, 100] (assuming motor.setSpeed takes -100 to 100)
        insertion_speed = std::max(-100.0f, std::min(100.0f, insertion_speed));

        // Apply insertion control
        motor3.setSpeed(static_cast<int16_t>(insertion_speed)); // Apply clamped speed
        commanded_speeds[2] = insertion_speed; // Store commanded insertion speed for telemetry



    // Update commanded_positions for telemetry based on active mode
    // This ensures that `joint_telemetry_msg.effort.data[0-2]` (which is mapped to commanded_positions)
    // reflects the commanded value from either ROS or sine wave.
    if (control_mode == "sin") {
        commanded_positions[0] = target_roll_angle;
        commanded_positions[1] = target_pitch_angle;
        commanded_positions[2] = target_insertion_position;
    } // If "target" mode, commanded_positions are already updated by target_pose_callback

    // 5. Read raw sensor data
    read_encoder_data(&sensor_data_msg);
    RCSOFTCHECK(rcl_publish(&sensor_data_publisher, &sensor_data_msg, NULL));

    // 6. Process incoming ROS messages (this updates received_joint_state and target_pose_msg)
    RCSOFTCHECK(rclc_executor_spin_some(&executor, 10));

    // 7. Publish telemetry
    publish_joint_telemetry(actual_positions, commanded_positions, commanded_speeds);

    // 8. Maintain loop timing
    delay(2); // Sampling rate of 500 Hz (2 ms per iteration)
}

// ----------------------
// PRBS Characterization, code is commented out to avoid confusion with the main control loop.
// This code is for testing the PRBS control algorithm and should not be used in production.
// Uncomment to use for testing PRBS control algorithm.
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
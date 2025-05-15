#include <Arduino.h>
#include "config.h"


// ----------------------
// Global ROS 2 Objects
// ----------------------
rcl_subscription_t joint_state_subscriber;
rcl_subscription_t target_pose_subscriber;
rcl_publisher_t sensor_data_publisher;
rcl_publisher_t debug_publisher;
rcl_publisher_t joint_telemetry_publisher;  // New telemetry publisher

sensor_msgs__msg__JointState received_joint_state;
geometry_msgs__msg__PoseStamped target_pose_msg;
std_msgs__msg__Int32MultiArray sensor_data_msg;
std_msgs__msg__String debug_msg;
sensor_msgs__msg__JointState joint_telemetry_msg;  // New telemetry message

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
// ROS Callbacks
// ----------------------
void joint_state_callback(const void *msgin) {
    const sensor_msgs__msg__JointState *msg = (const sensor_msgs__msg__JointState *)msgin;

    if (msg->position.size < 7 || msg->position.data == NULL) {
        publish_debug_message("[WARN] Invalid joint state message.");
        return;
    }

    float g0 = msg->position.data[6];
    float g1 = msg->position.data[5];
    float g2 = msg->position.data[4];
    float g3 = msg->position.data[3];

    g3 = map_gimbal_to_servo(g3, G3_MIN, G3_MAX, -180, 180);
    g2 = map_gimbal_to_servo(g2, G2_MIN, G2_MAX, -90, 90);
    g1 = map_gimbal_to_servo(g1, G1_MIN, G1_MAX, -90, 90);
    g0 = map_gimbal_to_servo(g0, G0_MIN, G0_MAX, -90, 90);

    servo_val[0] = -g0 / 2 + g1 + servo_off[0];
    servo_val[1] = g0 / 2 + g1 + servo_off[1];
    servo_val[2] = g2 + servo_off[2];
    servo_val[3] = g3 + servo_off[3];

    servo1.write(servo_val[0]);
    servo2.write(servo_val[1]);
    servo3.write(servo_val[2]);
    servo4.write(servo_val[3]);
}

void target_pose_callback(const void *msgin) {
    const geometry_msgs__msg__PoseStamped *msg = (const geometry_msgs__msg__PoseStamped *)msgin;

    // Directly copy into your commanded_positions buffer
    commanded_positions[0] = msg->pose.position.x;
    commanded_positions[1] = msg->pose.position.y;
    commanded_positions[2] = msg->pose.position.z;

}

// ----------------------
// Telemetry Functions
// ----------------------
void init_joint_telemetry_message() {
    memset(&joint_telemetry_msg, 0, sizeof(sensor_msgs__msg__JointState));
    
    // Initialize joint names
    const char* joint_names[] = {"roll", "pitch", "insertion"};
    joint_telemetry_msg.name.data = (rosidl_runtime_c__String *)malloc(3 * sizeof(rosidl_runtime_c__String));
    joint_telemetry_msg.name.size = 3;
    joint_telemetry_msg.name.capacity = 3;
    
    for (size_t i = 0; i < 3; i++) {
        rosidl_runtime_c__String__init(&joint_telemetry_msg.name.data[i]);
        joint_telemetry_msg.name.data[i].data = (char *)malloc(strlen(joint_names[i]) + 1); // Allocate exact size
        joint_telemetry_msg.name.data[i].size = strlen(joint_names[i]);
        joint_telemetry_msg.name.data[i].capacity = strlen(joint_names[i]) + 1;
        strcpy(joint_telemetry_msg.name.data[i].data, joint_names[i]); // Copy the name
    }

    // Initialize arrays
    joint_telemetry_msg.position.data = (double *)malloc(3 * sizeof(double));
    joint_telemetry_msg.position.size = 3;
    joint_telemetry_msg.position.capacity = 3;

    joint_telemetry_msg.velocity.data = (double *)malloc(3 * sizeof(double));
    joint_telemetry_msg.velocity.size = 3;
    joint_telemetry_msg.velocity.capacity = 3;

    joint_telemetry_msg.effort.data = (double *)malloc(3 * sizeof(double));
    joint_telemetry_msg.effort.size = 3;
    joint_telemetry_msg.effort.capacity = 3;
}

void publish_joint_telemetry(double* actual_positions, double* commanded_positions, double* commanded_speeds) {
    // Update message data
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

    // Hardware setup
    pinMode(LS1_NO, INPUT_PULLUP);
    pinMode(LS2_NO, INPUT_PULLUP);
    pinMode(LS3_NO, INPUT_PULLUP);
    pinMode(LS1_NC, INPUT_PULLUP);
    pinMode(LS2_NC, INPUT_PULLUP);
    pinMode(LS3_NC, INPUT_PULLUP);

    servo1.attach(SERVO1_PIN);
    servo2.attach(SERVO2_PIN);
    servo3.attach(SERVO3_PIN);
    servo4.attach(SERVO4_PIN);
    servo1.write(servo_off[0]);
    servo2.write(servo_off[1]);
    servo3.write(servo_off[2]);
    servo4.write(servo_off[3]);

    float PWM_freq = 18500.0;
    analogWriteFrequency(DC1_PWM, PWM_freq);
    analogWriteFrequency(DC2_PWM, PWM_freq);
    analogWriteFrequency(DC3_PWM, PWM_freq);

    // ROS 2 initialization
    allocator = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "psm_sensor_node", "", &support));

    // Publishers and Subscribers
    RCCHECK(rclc_subscription_init_default(
        &joint_state_subscriber, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
        "/mtm_joint_states"));

    rcl_subscription_options_t subscription_options = rcl_subscription_get_default_options();
    subscription_options.qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE; // Ensure reliable delivery
    subscription_options.qos.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;   // No need for late-joining subscribers
    subscription_options.qos.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;        // Keep only the last few messages
    subscription_options.qos.depth = 20;                                       // Queue depth of 10 messages
    subscription_options.qos.deadline.sec = 0;                                 // Deadline of 10 ms
    subscription_options.qos.deadline.nsec = 10000000;                         // (10 ms in nanoseconds)
    subscription_options.qos.liveliness = RMW_QOS_POLICY_LIVELINESS_AUTOMATIC; // Default liveliness

    RCCHECK(rcl_subscription_init(
        &target_pose_subscriber, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, PoseStamped),
        "/target_pose", &subscription_options));

    RCCHECK(rclc_publisher_init_default(
        &sensor_data_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
        "/psm_sensor_data"));

    RCCHECK(rclc_publisher_init_default(
        &debug_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "/psm_debug"));

    // Initialize telemetry publisher
    RCCHECK(rclc_publisher_init_default(
        &joint_telemetry_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
        "/psm_joint_telemetry"));

    // Message memory allocation
    sensor_data_msg.data.data = (int32_t *)malloc(3 * sizeof(int32_t));
    sensor_data_msg.data.size = 3;
    sensor_data_msg.data.capacity = 3;

    debug_msg.data.data = (char *)malloc(128 * sizeof(char));
    debug_msg.data.size = 0;
    debug_msg.data.capacity = 128;

    init_joint_telemetry_message();

    // Executor setup
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator)); 
    RCCHECK(rclc_executor_add_subscription(
        &executor, &joint_state_subscriber, &received_joint_state, &joint_state_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(
        &executor, &target_pose_subscriber, &target_pose_msg, &target_pose_callback, ON_NEW_DATA));

    home_motors();
    delay(2500); // Delay for ease of use

 }

// ----------------------
// Loop Function
// ----------------------
void loop() {
    static unsigned long start_time = millis(); // Record the start time of the sine wave
    static bool sine_wave_started = false;

    // Calculate elapsed time since the program started
    unsigned long current_time = millis();
    float elapsed_time = (current_time - start_time) / 1000.0f; // Convert to seconds

    // 1. Read filtered encoder values
    actual_positions[0] = Ax1toAngle(Enc1.read()); // Roll
    actual_positions[1] = Ax2toAngle(Enc2.read()); // Pitch
    actual_positions[2] = Ax3toAngle(Enc3.read()); // Insertion

    // == Pitch Axis Sine Wave ==
    if (!sine_wave_started) {
        sine_wave_started = true;
        start_time = millis(); // Reset start time for sine wave
    }

    // Calculate the sine wave target position
    const float amplitude = 5.0f; // Amplitude of the sine wave (±15 degrees)
    const float period = 5.0f;     // Period of the sine wave (5 seconds)1
    float target_position = amplitude * sin((2.0f * PI / period) * elapsed_time);
    // target_position = 0.0f;
    commanded_positions[1] = target_position; // Store target position for telemetry

    // Use the LQI controller to move the pitch axis to the target position
    float pitch_lqi_gains[3] = {220.6809f, 0.3174f, 0.1414f}; // Gains for position error, velocity, and integral
    float pitch_speed = compute_pitch_LQI_control(
        pitch_lqi_gains, 
        target_position, // Target position for pitch
        actual_positions[1] // Actual position (pitch)
    );
    motor2.setSpeed(static_cast<int16_t>(pitch_speed)); // Apply LQI control to motor2
    commanded_speeds[1] = pitch_speed; // Store commanded speed for telemetry

    // == Roll Axis LQR ==
    float roll_lqr_gains[2] = {200.1886f, 2.0860f}; // Gains for position error and velocity
    float roll_speed = compute_roll_LQR_control(
        roll_lqr_gains, 
        0.0f, // Target position for roll is 0 degrees
        actual_positions[0] // Actual position (roll)
    );
    motor1.setSpeed(static_cast<int16_t>(roll_speed));
    commanded_speeds[0] = roll_speed; // Store commanded speed for telemetry

    // 2. Read raw sensor data (no filtering)
    read_encoder_data(&sensor_data_msg);
    RCSOFTCHECK(rcl_publish(&sensor_data_publisher, &sensor_data_msg, NULL));

    // 3. Process incoming ROS messages
    RCSOFTCHECK(rclc_executor_spin_some(&executor, 10));

    // 4. Publish telemetry
    publish_joint_telemetry(actual_positions, commanded_positions, commanded_speeds);

    // 5. Maintain loop timing
    delay(2); // Sampling rate of 500 Hz (2 ms per iteration)
}

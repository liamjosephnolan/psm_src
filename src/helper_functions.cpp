#include "config.h"

// ----------------------
// Debugging Functions
// ----------------------

// Publishes a debug message to the ROS 2 debug topic
void publish_debug_message(const char *message) {
    // Assuming debug_msg and debug_publisher are defined globally elsewhere (or declared extern in config.h)
    snprintf(debug_msg.data.data, debug_msg.data.capacity, "%s", message);
    debug_msg.data.size = strlen(debug_msg.data.data);
    RCSOFTCHECK(rcl_publish(&debug_publisher, &debug_msg, NULL));
}

// ----------------------
// Servo Mapping Functions
// ----------------------

// Maps a gimbal angle to a servo angle and constrains it within valid servo limits
int map_gimbal_to_servo(double gimbal_angle, double gimbal_min, double gimbal_max, double max_angle, double min_angle) {
    int servo_angle = map((int)gimbal_angle, gimbal_min, gimbal_max, max_angle, min_angle);
    return constrain(servo_angle, SERVO_ANGLE_MIN, SERVO_ANGLE_MAX);
}

// ----------------------
// Error Handling Functions
// ----------------------

// Enters an infinite loop to indicate a critical error
void error_loop() {
    while (1) {
        delay(1000);  // Simple error loop
    }
}
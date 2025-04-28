#include "config.h"

// ----------------------
// External Dependencies
// ----------------------
extern CytronMD motor[3];
extern Encoder Enc1;
extern Encoder Enc2;
extern Encoder Enc3;
extern void publish_debug_message(const char *message);

// ----------------------
// Constants
// ----------------------
const int MOTOR1_MAX_SPEED = 40;
const int MOTOR3_SPEED = 500;
const int MOTOR1_BACKOFF_TIME = 600; // Milliseconds
const int MOTOR3_BACKOFF_TIME = 500; // Milliseconds

// ----------------------
// Helper Functions
// ----------------------

// Publishes a debug message with motor-specific information
void debug_log(const char *prefix, int motor_index, const char *msg) {
    char debug_msg[64];
    snprintf(debug_msg, sizeof(debug_msg), "[Motor %d] %s: %s", motor_index + 1, prefix, msg);
    publish_debug_message(debug_msg);
}

// ----------------------
// Homing Routines
// ----------------------

// Homing routine for Motor 1 — slow ramp + backoff
void home_motor1() {
    debug_log("Start", 0, "Homing with speed ramp...");

    int speed = 10;
    const int step = 5;
    const int delay_step = 150;

    // Ramp up until limit switch triggers or speed hits max
    while (digitalRead(LS1_NO) == HIGH) {
        motor[0].setSpeed(speed);
        delay(delay_step);
        speed += step;
        if (speed > MOTOR1_MAX_SPEED) {
            speed = MOTOR1_MAX_SPEED;
        }
    }

    motor[0].setSpeed(0);
    Enc1.write(0);
    debug_log("Info", 0, "Limit switch hit. Encoder zeroed.");

    // Back off slowly
    motor[0].setSpeed(20); // Lower than approach speed
    delay(MOTOR1_BACKOFF_TIME);
    motor[0].setSpeed(0);

    debug_log("Done", 0, "Homing complete.");
}

// Homing routine for Motor 2 — encoder reset only
void home_motor2() {
    debug_log("Info", 1, "Already on limit switch. Zeroing encoder only.");
    Enc2.write(0);
    debug_log("Done", 1, "Encoder zeroed.");
}

// Homing routine for Motor 3 — fixed speed approach
void home_motor3() {
    debug_log("Start", 2, "Driving toward limit switch...");

    motor[2].setSpeed(MOTOR3_SPEED);
    while (digitalRead(LS3_NO) == HIGH) {
        delay(10);
    }

    motor[2].setSpeed(0);
    Enc3.write(0);
    debug_log("Info", 2, "Limit switch hit. Encoder zeroed.");

    // Optional backoff
    motor[2].setSpeed(-25);
    delay(MOTOR3_BACKOFF_TIME);
    motor[2].setSpeed(0);

    debug_log("Done", 2, "Homing complete.");
}

// ----------------------
// Master Homing Function
// ----------------------

// Master function to home all motors
void home_motors() {
    publish_debug_message("Starting homing sequence...");

    home_motor1();
    home_motor2();
    home_motor3();

    publish_debug_message("All motors homed successfully.");
}

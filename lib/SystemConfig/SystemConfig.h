#ifndef SYSTEM_CONFIG_H
#define SYSTEM_CONFIG_H

#include <Arduino.h>

// ----- Pin and Hardware Configuration -----
#define PIN            8    // NeoPixel data pin
#define NUMPIXELS      1    // Number of NeoPixels
#define SPEKTRUM_SERIAL Serial1  // Serial port for Spektrum satellite receiver
#define BIND_MODE_PIN  A0   // Pin to check for bind mode (connect to ground to enable bind mode)
#define SPEKTRUM_RX_PIN 0   // Pin 0 (D0/RX) - used for both bind pulses and serial data
#define MOTOR_ID_1     1  // First motor ID
#define MOTOR_ID_2     2  // Second motor ID
#define MASTER_ID      0xFD // Default Master ID for commands

// ----- Spektrum Satellite Configuration -----
#define MIN_PULSE      1000 // 1000us = -20 rad/s
#define MID_PULSE      1500 // 1500us = dead zone
#define MAX_PULSE      2000 // 2000us = 20 rad/s
#define DEAD_ZONE      50   // ±50us dead zone around 1500us

// crsf Channel
#define CH_DRIVE_L 1    // Unused for the arms
#define CH_DRIVE_R 2    // Unused for the arms
#define CH_TRIM_L 3     // Position Trim Left arm
#define CH_TRIM_R 4     // Position Trim Right arm
#define CH_ESTOP 5        // Armed/Estop Channel
#define CH_MODE 6       // Treated as a 4 position switch to set control mode
#define CH_ANALOG_SWITCH 7  // Treated as a 4 position switch
#define CH_L_ARM 8      // Directly sets position (with trim adjustment) or velocity depending on mode
#define CH_R_ARM 9      // Directly sets position (with trim adjustment) or velocity depending on mode

// ----- Bind Mode Configuration -----
#define BIND_ENABLED false  // Set to true to enable bind mode

// ----- Mode Selection Thresholds -----
#define MODE_THRESHOLD 1800 // Above 1800us = position mode, below = velocity mode
#define ZERO_THRESHOLD 1800 // Above 1800us = set position to zero
#define SELECT_HIGH    1800 // Above 1800us = motor 1 only
#define SELECT_LOW     1300 // Below 1300us = motor 2 only
                            // Between 1300-1800 = both motors, motor 2 reversed

// ----- Motor Configuration -----
#define MOTOR_CURRENT_LIMIT 40.0f   // Current limit in amperes
#define MOTOR_ACCELERATION  200.0f  // Acceleration limit in rad/s² for velocity mode
#define POSITION_SPEED_LIMIT 25.0f  // Speed limit for position mode in rad/s
#define POSITION_ACCELERATION 200.0f // Acceleration limit in rad/s² for position mode
#define MAX_VELOCITY        25.0f   // Maximum motor velocity in rad/s
#define MIN_POSITION        -6.28f   // Minimum position in radians
#define MAX_POSITION        6.28f    // Maximum position in radians

// ----- Timing Configuration -----
#define MOTOR_UPDATE_RATE_MS 20     // 20ms = 50 updates per second
#define DEBUG_PRINT_INTERVAL 500    // Print debug info every 500ms
#define FEEDBACK_INTERVAL    200    // Fetch and report motor feedback every 200ms
#define DISPLAY_UPDATE_RATE_MS 150  // Update display every 150ms

// ----- OLED Display Configuration -----
#define SCREEN_WIDTH 128     // OLED display width, in pixels
#define SCREEN_HEIGHT 64     // OLED display height, in pixels
#define OLED_RESET     -1    // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C  // I2C address of the OLED display
#define DISPLAY_UPDATE_RATE_MS 150  // Update display every 150ms

// ----- Debug Levels -----
#define LOG_ERROR   1
#define LOG_WARNING 2
#define LOG_INFO    3
#define LOG_DEBUG   4
#define LOG_VERBOSE 5

// Set current debug level
#define DEBUG_LEVEL LOG_INFO

// Motor selection enum for clarity
enum MotorSelection {
    MOTOR_1_ONLY,     // Motor 1 only (ID 127)
    MOTOR_2_ONLY,     // Motor 2 only (ID 126)
    BOTH_MOTORS       // Both motors, with motor 2 reversed
};

enum State {
    TO_ESTOP,
    ESTOP,
    TO_MODE_CONTROL,        // Make sure things initialize out of ESTOP properly
    VELOCITY_MODE,          // Previously Mode 1
    POSITION_SETPOINT_MODE, // Previously Mode 3
    POSITION_STOW_MODE,     // Previously Mode 4
    POSITION_ANALOG_MODE,   // Previously Mode 5


};

// Define a structure to hold raw feedback data for each motor
struct MotorRawFeedback {
    float position = 0.0f;
    float velocity = 0.0f;
    float torque = 0.0f;
    float temperature = 0.0f;
    float current = 0.0f;
    float currentLimit = 0.0f;
    bool received = false;
    unsigned long timestamp = 0;
};

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#endif // SYSTEM_CONFIG_H 
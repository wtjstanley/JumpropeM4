#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <Arduino.h>
#include "SystemConfig.h"
#include "RS03Motor.h"
#include "Logger.h"
#include <Adafruit_NeoPixel.h>

class MotorController {
private:
    RS03Motor& motor1;
    RS03Motor& motor2;
    Adafruit_NeoPixel& pixels;
    
    // State variables
    bool motorsInitialized;
    bool inPositionMode;
    bool modeInitialized;
    MotorSelection currentMotorSelection;
    float currentPosition;
    float rcCurrentLimit;
    
    // Zero position management
    unsigned long lastZeroSetTime;
    bool holdingZeroPosition;
    unsigned long zeroHoldStartTime;
    
    // Timing
    unsigned long lastMotorUpdateTime;
    unsigned long lastFeedbackTime;
    unsigned long lastDebugPrint;
    
    // Feedback storage
    MotorRawFeedback motor1_feedback;
    MotorRawFeedback motor2_feedback;
    
public:
    MotorController(RS03Motor& m1, RS03Motor& m2, Adafruit_NeoPixel& pix);
    
    // Initialization
    bool begin();
    
    // Main update loop
    void update();
    
    // Motor initialization helpers
    void initializeMotor(RS03Motor& motor);
    void initializeMotorsForMode(bool positionMode);
    void initializePositionMode(RS03Motor& motor);
    void initializeVelocityMode(RS03Motor& motor);
    
    // Feedback and CAN processing
    void processCanMessages();
    void fetchAndReportMotorFeedback(RS03Motor& motor, const String& motorLabel);
    bool queryParameter(uint16_t index, float &value, uint8_t motor_id);
    
    // LED control
    void setAllPixelsColor(int red, int green, int blue);
    
    // Getters for display
    bool isInPositionMode() const { return inPositionMode; }
    MotorSelection getCurrentMotorSelection() const { return currentMotorSelection; }
    const MotorRawFeedback& getMotor1Feedback() const { return motor1_feedback; }
    const MotorRawFeedback& getMotor2Feedback() const { return motor2_feedback; }
    bool areMotorsInitialized() const { return motorsInitialized; }
};

#endif // MOTOR_CONTROLLER_H 
#include "MotorController.h"
#include <CANSAME5x.h>
#include <cmath>

extern CANSAME5x CAN;

MotorController::MotorController(RS03Motor& m1, RS03Motor& m2, Adafruit_NeoPixel& pix)
    : motor1(m1), motor2(m2), pixels(pix),
      motorsInitialized(false), inPositionMode(false), modeInitialized(false),
      currentMotorSelection(MOTOR_1_ONLY), currentPosition(0.0f), rcCurrentLimit(MOTOR_CURRENT_LIMIT),
      lastZeroSetTime(0), holdingZeroPosition(false), zeroHoldStartTime(0),
      lastMotorUpdateTime(0), lastFeedbackTime(0), lastDebugPrint(0) {
}

bool MotorController::begin() {
    Logger::info("Initializing motors with IDs: " + String(MOTOR_ID_1) + " and " + String(MOTOR_ID_2));
    
    // Initialize both motors
    initializeMotor(motor1);
    initializeMotor(motor2);
    
    // Set the motors to velocity mode initially
    initializeMotorsForMode(false);  // false = velocity mode
    inPositionMode = false;
    
    motorsInitialized = true;
    setAllPixelsColor(0, 50, 0); // Green indicates ready
    Logger::info("Motor controller initialization complete");
    
    return true;
}

/*
void MotorController::update() {
    // Update RC input
    rcInput.update();
    
    // Process any incoming CAN messages to update feedback
    for (int i = 0; i < 5; i++) {
        processCanMessages();
        delayMicroseconds(100);
    }
    
    // Periodically request motor state directly using Type 0 query
    static unsigned long lastDirectQueryTime = 0;
    unsigned long currentTime = millis();
    if (currentTime - lastDirectQueryTime >= 100) { // Every 100ms
        // Send a Type 0 query to request state from both motors based on current selection
        if (currentMotorSelection == MOTOR_1_ONLY || currentMotorSelection == BOTH_MOTORS) {
            uint32_t queryId1 = ((0x00) << 24) | (MASTER_ID << 16) | MOTOR_ID_1;
            if (CAN.beginExtendedPacket(queryId1 & 0x1FFFFFFF)) {
                CAN.endPacket();
            }
        }
        
        if (currentMotorSelection == MOTOR_2_ONLY || currentMotorSelection == BOTH_MOTORS) {
            uint32_t queryId2 = ((0x00) << 24) | (MASTER_ID << 16) | MOTOR_ID_2;
            if (CAN.beginExtendedPacket(queryId2 & 0x1FFFFFFF)) {
                CAN.endPacket();
            }
        }
        
        lastDirectQueryTime = currentTime;
    }

    static unsigned long lastChannelPrint = 0;
    
    // Print all channel values once per second
    if (currentTime - lastChannelPrint >= 1000) {
        String channelValues = "Spektrum Channels: ";
        for (int i = 0; i < SPEKTRUM_CHANNELS; i++) {
            channelValues += String(i + 1) + "=" + String(rcInput.getVelocityChannel()) + "us ";
        }
        Logger::debug(channelValues);
        
        // Also print signal status
        Logger::debug("Signal Status: " + String(rcInput.isReceiving() ? "RECEIVING" : "NO SIGNAL"));
        
        lastChannelPrint = currentTime;
    }

    // Handle motor control logic
    if (motorsInitialized) {
        // Check which motor(s) to control based on SELECT_CHANNEL
        MotorSelection newMotorSelection = rcInput.getMotorSelection();
        
        // Log if the motor selection changed
        if (newMotorSelection != currentMotorSelection) {
            String selectionText;
            switch (newMotorSelection) {
                case MOTOR_1_ONLY: selectionText = "Motor 1 only (ID " + String(MOTOR_ID_1) + ")"; break;
                case MOTOR_2_ONLY: selectionText = "Motor 2 only (ID " + String(MOTOR_ID_2) + ")"; break;
                case BOTH_MOTORS: selectionText = "Both motors (Motor 2 reversed)"; break;
            }
            Logger::info("Motor selection changed: " + selectionText);
            currentMotorSelection = newMotorSelection;
        }
        
        // Check if we need to switch control modes
        bool shouldBeInPositionMode = rcInput.shouldBeInPositionMode();
        
        // Only switch modes if the mode has changed
        if (shouldBeInPositionMode != inPositionMode) {
            inPositionMode = shouldBeInPositionMode;
            modeInitialized = false; // Need to re-initialize the mode
            
            Logger::info(inPositionMode ? "Will switch to position mode" : "Will switch to velocity mode");
        }
        
        // Initialize the appropriate mode if needed
        if (!modeInitialized) {
            initializeMotorsForMode(inPositionMode);
        }
        
        // Handle zero position logic
        if (rcInput.shouldZeroPosition() && !holdingZeroPosition && (currentTime - lastZeroSetTime >= 500)) {
            
            if (inPositionMode) {
                Logger::info("Setting current position as zero in position mode");
            } else {
                Logger::info("Setting current position as zero in velocity mode");
            }
            
            // Set mechanical zero based on current motor selection
            bool zeroSuccess = true;
            
            if (currentMotorSelection == MOTOR_1_ONLY || currentMotorSelection == BOTH_MOTORS) {
                if (!motor1.setMechanicalZero()) {
                    Logger::error("Failed to set mechanical zero for motor 1!");
                    zeroSuccess = false;
                }
            }
            
            if (currentMotorSelection == MOTOR_2_ONLY || currentMotorSelection == BOTH_MOTORS) {
                if (!motor2.setMechanicalZero()) {
                    Logger::error("Failed to set mechanical zero for motor 2!");
                    zeroSuccess = false;
                }
            }
            
            if (zeroSuccess) {
                currentPosition = 0.0f;
                
                if (inPositionMode) {
                    holdingZeroPosition = true;
                    zeroHoldStartTime = currentTime;
                    Logger::info("Entering zero hold position mode");
                }
                
                lastZeroSetTime = currentTime;
            }
        }
        
        // Handle zero hold exit logic
        if (holdingZeroPosition) {
            float desiredPosition = rcInput.mapPulseToPosition(rcInput.getPositionChannel());
            
            if ((currentTime - zeroHoldStartTime > 1000) || (fabs(desiredPosition) < 0.1f)) {
                Logger::info("Exiting zero hold position mode");
                holdingZeroPosition = false;
            } else {
                currentPosition = 0.0f;
                
                // Set the position to zero for the active motor(s)
                if (currentMotorSelection == MOTOR_1_ONLY || currentMotorSelection == BOTH_MOTORS) {
                    motor1.setPosition(0.0f);
                }
                
                if (currentMotorSelection == MOTOR_2_ONLY || currentMotorSelection == BOTH_MOTORS) {
                    motor2.setPosition(0.0f);
                }
            }
        }
        
        // Update motor at the specified rate
        if (currentTime - lastMotorUpdateTime >= MOTOR_UPDATE_RATE_MS) {
            // If no signal is detected, default to center position (stop/neutral)
            if (!rcInput.isReceiving()) {
                Logger::error("No Spektrum signal detected!");
                motor1.setVelocity(0);
                motor2.setVelocity(0);
                setAllPixelsColor(50, 50, 0); // Yellow for no signal
            } else {
                if (inPositionMode && modeInitialized) {
                    // Position Mode
                    float targetPosition = rcInput.mapPulseToPosition(rcInput.getPositionChannel());
                    
                    if (!holdingZeroPosition) {
                        currentPosition = targetPosition;
                    }
                    
                    // Print debug info occasionally
                    if (currentTime - lastDebugPrint >= DEBUG_PRINT_INTERVAL) {
                        Logger::info("Position Mode: Target Position: " + String(holdingZeroPosition ? 0.0f : targetPosition) + " rad");
                        Logger::info(rcInput.getAllChannelValues());
                        lastDebugPrint = currentTime;
                    }
                    
                    float positionToSet = holdingZeroPosition ? 0.0f : targetPosition;
                    bool success = true;
                    
                    // For motor 1
                    if (currentMotorSelection == MOTOR_1_ONLY || currentMotorSelection == BOTH_MOTORS) {
                        if (!motor1.setPosition(positionToSet)) {
                            Logger::error("Failed to set position for motor 1!");
                            success = false;
                        }
                    }
                    
                    // For motor 2 (with reversed direction when in BOTH_MOTORS mode)
                    if (currentMotorSelection == MOTOR_2_ONLY || currentMotorSelection == BOTH_MOTORS) {
                        float motor2Position = positionToSet;
                        
                        if (currentMotorSelection == BOTH_MOTORS) {
                            motor2Position = -positionToSet;
                        }
                        
                        if (!motor2.setPosition(motor2Position)) {
                            Logger::error("Failed to set position for motor 2!");
                            success = false;
                        }
                    }
                    
                    // Set LED color based on position and success
                    if (success) {
                        if (holdingZeroPosition) {
                            int pulseIntensity = (millis() / 100) % 50;
                            setAllPixelsColor(pulseIntensity, pulseIntensity, pulseIntensity);
                        } else if (positionToSet > 0) {
                            int intensity = map(abs(positionToSet * 1000), 0, MAX_POSITION * 1000, 0, 50);
                            setAllPixelsColor(0, intensity, intensity);
                        } else if (positionToSet < 0) {
                            int intensity = map(abs(positionToSet * 1000), 0, MAX_POSITION * 1000, 0, 50);
                            setAllPixelsColor(intensity, 0, intensity);
                        } else {
                            setAllPixelsColor(20, 20, 20);
                        }
                    } else {
                        setAllPixelsColor(50, 50, 0); // Yellow for command error
                    }
                    
                } else if (!inPositionMode && modeInitialized) {
                    // Velocity Mode
                    float targetVelocity = rcInput.mapPulseToVelocity(rcInput.getVelocityChannel());
                    float currentLimit = rcInput.mapPulseToCurrentLimit(rcInput.getCurrentChannel());
                    
                    rcCurrentLimit = currentLimit;
                    
                    // Print debug info occasionally
                    if (currentTime - lastDebugPrint >= DEBUG_PRINT_INTERVAL) {
                        Logger::info("Velocity Mode: Velocity: " + String(targetVelocity) + " rad/s, " +
                                      "Current Limit: " + String(currentLimit) + " A");
                        Logger::info(rcInput.getAllChannelValues());
                        lastDebugPrint = currentTime;
                    }
                    
                    bool success = true;
                    
                    // Set velocity for the appropriate motor(s)
                    if (currentMotorSelection == MOTOR_1_ONLY || currentMotorSelection == BOTH_MOTORS) {
                        if (!motor1.setVelocityWithLimits(targetVelocity, currentLimit, MOTOR_ACCELERATION)) {
                            Logger::error("Failed to set velocity for motor 1!");
                            success = false;
                        }
                    }
                    
                    // For motor 2 (with reversed direction when in BOTH_MOTORS mode)
                    if (currentMotorSelection == MOTOR_2_ONLY || currentMotorSelection == BOTH_MOTORS) {
                        float motor2Velocity = targetVelocity;
                        
                        if (currentMotorSelection == BOTH_MOTORS) {
                            motor2Velocity = -targetVelocity;
                        }
                        
                        if (!motor2.setVelocityWithLimits(motor2Velocity, currentLimit, MOTOR_ACCELERATION)) {
                            Logger::error("Failed to set velocity for motor 2!");
                            success = false;
                        }
                    }
                    
                    // Set LED color based on velocity and success
                    if (success) {
                        if (targetVelocity > 0) {
                            int intensity = map(abs(targetVelocity * 1000), 0, MAX_VELOCITY * 1000, 0, 50);
                            setAllPixelsColor(0, intensity, 0);
                        } else if (targetVelocity < 0) {
                            int intensity = map(abs(targetVelocity * 1000), 0, MAX_VELOCITY * 1000, 0, 50);
                            setAllPixelsColor(intensity, 0, 0);
                        } else {
                            setAllPixelsColor(0, 0, 20);
                        }
                    } else {
                        setAllPixelsColor(50, 50, 0); // Yellow for command error
                    }
                }
            }
            
            lastMotorUpdateTime = currentTime;
        }
        
        // Fetch and report motor feedback
        if (currentTime - lastFeedbackTime >= FEEDBACK_INTERVAL) {
            if (currentMotorSelection == MOTOR_1_ONLY) {
                fetchAndReportMotorFeedback(motor1, "MOTOR 1");
            } else if (currentMotorSelection == MOTOR_2_ONLY) {
                fetchAndReportMotorFeedback(motor2, "MOTOR 2");
            } else { // BOTH_MOTORS
                fetchAndReportMotorFeedback(motor1, "MOTOR 1");
                fetchAndReportMotorFeedback(motor2, "MOTOR 2");
            }
            
            lastFeedbackTime = currentTime;
        }
    }
}
*/
void MotorController::initializeMotor(RS03Motor& motor) {
    Logger::info("Initializing motor with ID: " + String(motor.getMotorId()));
    
    // Reset any faults
    Logger::info("Resetting motor faults for ID " + String(motor.getMotorId()) + "...");
    if (!motor.resetFaults()) {
        Logger::error("Failed to reset motor faults for ID " + String(motor.getMotorId()) + "!");
    } else {
        Logger::info("Motor faults reset for ID " + String(motor.getMotorId()));
    }
    delay(100);
    
    // Enable the motor
    Logger::info("Enabling motor ID " + String(motor.getMotorId()) + "...");
    if (!motor.enable()) {
        Logger::error("Failed to enable motor ID " + String(motor.getMotorId()) + "!");
    } else {
        Logger::info("Motor ID " + String(motor.getMotorId()) + " enabled");
    }
    delay(100);
    
    // Enable active reporting from the motor
    Logger::info("Enabling active reporting from motor ID " + String(motor.getMotorId()) + "...");
    
    bool reportingEnabled = motor.setActiveReporting(true);
    
    if (!reportingEnabled) {
        Logger::warning("Library method failed, trying direct approach for active reporting for motor ID " + String(motor.getMotorId()));
        
        // Type 5 message for active reporting command
        uint32_t reportingId = ((static_cast<uint32_t>(0x05) << 24) | 
                              (static_cast<uint32_t>(MASTER_ID) << 8) | 
                              static_cast<uint32_t>(motor.getMotorId())) & 0x1FFFFFFF;
        
        uint8_t reportingData[8] = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // 0x01 = enable reporting
        
        if (CAN.beginExtendedPacket(reportingId)) {
            CAN.write(reportingData, 8);
            if (CAN.endPacket()) {
                Logger::info("Direct active reporting command sent for motor ID " + String(motor.getMotorId()));
                reportingEnabled = true;
            } else {
                Logger::error("Failed to send direct active reporting command for motor ID " + String(motor.getMotorId()));
            }
        } else {
            Logger::error("Failed to begin direct active reporting packet for motor ID " + String(motor.getMotorId()));
        }
    }
    
    if (reportingEnabled) {
        Logger::info("Active reporting enabled for motor ID " + String(motor.getMotorId()));
    } else {
        Logger::error("Failed to enable active reporting for motor ID " + String(motor.getMotorId()));
    }
}

void MotorController::initializeMotorsForMode(bool positionMode) {
    if (positionMode) {
        Logger::info("Initializing all motors in position mode");
        initializePositionMode(motor1);
        initializePositionMode(motor2);
    } else {
        Logger::info("Initializing all motors in velocity mode");
        initializeVelocityMode(motor1);
        initializeVelocityMode(motor2);
    }
    modeInitialized = true;
}

void MotorController::initializePositionMode(RS03Motor& motor) {
    Logger::info("Initializing position mode (PP) with proper limits for motor ID " + String(motor.getMotorId()));
    
    if (!motor.setModePositionPP(POSITION_SPEED_LIMIT, POSITION_ACCELERATION, MOTOR_CURRENT_LIMIT)) {
        Logger::error("Failed to set position mode (PP) for motor ID " + String(motor.getMotorId()) + "!");
        return;
    }
    
    Logger::info("Position mode (PP) set for motor ID " + String(motor.getMotorId()) + 
                  " with speed limit: " + String(POSITION_SPEED_LIMIT) + 
                  " rad/s, acceleration: " + String(POSITION_ACCELERATION) + 
                  " rad/s², current limit: " + String(MOTOR_CURRENT_LIMIT) + " A");
}

void MotorController::initializeVelocityMode(RS03Motor& motor) {
    Logger::info("Initializing velocity mode for motor ID " + String(motor.getMotorId()));
    
    if (!motor.setModeVelocity()) {
        Logger::error("Failed to set velocity mode for motor ID " + String(motor.getMotorId()) + "!");
        return;
    }

    bool success = true;
    if (!motor.setParameterFloat(INDEX_LIMIT_CUR, MOTOR_CURRENT_LIMIT)) {
        Logger::error("Failed to set current limit for velocity mode for motor ID " + String(motor.getMotorId()) + "!");
        success = false;
    }
    
    if (!motor.setParameterFloat(0x7022, MOTOR_ACCELERATION)) {
        Logger::error("Failed to set acceleration limit for velocity mode for motor ID " + String(motor.getMotorId()) + "!");
        success = false;
    }
    
    if (success) {
        Logger::info("Velocity mode set for motor ID " + String(motor.getMotorId()) + 
                      " with current limit: " + String(MOTOR_CURRENT_LIMIT) + 
                      " A, acceleration: " + String(MOTOR_ACCELERATION) + " rad/s²");
    }
}

void MotorController::processCanMessages() {
    // Check for any available CAN messages
    int packetSize = CAN.parsePacket();
    if (packetSize) {
        // Create a CAN frame to hold the data
        CanFrame frame;
        frame.id = CAN.packetId();
        frame.is_extended = CAN.packetExtended();
        frame.dlc = packetSize;
        
        // Read data bytes
        int i = 0;
        while (CAN.available() && i < 8) {
            frame.data[i++] = CAN.read();
        }
        
        // Check specifically for Type 2 (feedback) messages
        uint8_t msgType = (frame.id >> 24) & 0x1F;
        if (msgType == 0x02 && frame.dlc == 8) {
            uint8_t sourceMotorId = (frame.id >> 8) & 0xFF;
            
            uint16_t pos_raw = (static_cast<uint16_t>(frame.data[0]) << 8) | frame.data[1];
            uint16_t vel_raw = (static_cast<uint16_t>(frame.data[2]) << 8) | frame.data[3];
            uint16_t torque_raw = (static_cast<uint16_t>(frame.data[4]) << 8) | frame.data[5];
            uint16_t temp_raw = (static_cast<uint16_t>(frame.data[6]) << 8) | frame.data[7];
            
            float position = motor1.uintToFloat(pos_raw, P_MIN, P_MAX, 16);
            float velocity = motor1.uintToFloat(vel_raw, V_MIN, V_MAX, 16);
            float torque = motor1.uintToFloat(torque_raw, T_MIN, T_MAX, 16);
            float temperature = static_cast<float>(temp_raw) / 10.0f;
            
            // Store in the appropriate motor's feedback structure
            if (sourceMotorId == MOTOR_ID_1) {
                motor1_feedback.position = position;
                motor1_feedback.velocity = velocity;
                motor1_feedback.torque = torque;
                motor1_feedback.temperature = temperature;
                motor1_feedback.received = true;
                motor1_feedback.timestamp = millis();
            } 
            else if (sourceMotorId == MOTOR_ID_2) {
                motor2_feedback.position = position;
                motor2_feedback.velocity = velocity;
                motor2_feedback.torque = torque;
                motor2_feedback.temperature = temperature;
                motor2_feedback.received = true;
                motor2_feedback.timestamp = millis();
            }
        }
        
        // Process the frame with both motors
        motor1.processFeedback(frame);
        motor2.processFeedback(frame);
    }
}

void MotorController::fetchAndReportMotorFeedback(RS03Motor& motor, const String& motorLabel) {
    float position = 0.0f;
    float velocity = 0.0f;
    float torque = 0.0f;
    float temperature = 0.0f;
    float current = 0.0f;
    bool success = false;
    
    uint8_t motorId = motor.getMotorId();
    
    Logger::info("Querying feedback from " + motorLabel + " (ID " + String(motorId) + ")");
    
    // Check if we have recent raw feedback from Type 2 messages for this motor
    bool useRawFeedback = false;
    unsigned long currentTime = millis();
    
    MotorRawFeedback* rawFeedback = nullptr;
    if (motorId == MOTOR_ID_1) {
        rawFeedback = &motor1_feedback;
    } else if (motorId == MOTOR_ID_2) {
        rawFeedback = &motor2_feedback;
    }
    
    // Check if we have recent raw feedback (less than 500ms old)
    if (rawFeedback && rawFeedback->received && (currentTime - rawFeedback->timestamp < 500)) {
        position = rawFeedback->position;
        velocity = rawFeedback->velocity;
        torque = rawFeedback->torque;
        temperature = rawFeedback->temperature;
        useRawFeedback = true;
        success = true;
        Logger::debug("Using raw feedback data from Type 2 messages for " + motorLabel);
    }
    
    // Query current (always query this)
    if (queryParameter(0x701A, current, motorId)) {
        success = true;
        Logger::debug("Direct current query success for " + motorLabel + ": " + String(current));
        
        if (rawFeedback) {
            rawFeedback->current = current;
        }
    }
    
    // If we don't have raw feedback, query all remaining parameters directly
    if (!useRawFeedback) {
        if (queryParameter(0x7000, position, motorId)) {
            success = true;
        }
        
        if (queryParameter(0x7001, velocity, motorId)) {
            success = true;
        }
        
        if (queryParameter(0x7002, torque, motorId)) {
            success = true;
        }
        
        if (queryParameter(0x7007, temperature, motorId)) {
            success = true;
        }
    }
    
    // If direct querying failed, use last feedback from the motor
    if (!success) {
        Logger::warning("Failed to query parameters directly for " + motorLabel + ", using last feedback");
        RS03Motor::Feedback feedback = motor.getLastFeedback();
        position = feedback.position;
        velocity = feedback.velocity;
        torque = feedback.torque;
        temperature = feedback.temperature;
    }
    
    // Report data
    Logger::info("-------- " + motorLabel + " FEEDBACK --------");
    Logger::info("Position: " + String(position) + " rad");
    Logger::info("Velocity: " + String(velocity) + " rad/s");
    Logger::info("Torque:   " + String(torque) + " Nm");
    Logger::info("Current:  " + String(current) + " A");
    Logger::info("Temp:     " + String(temperature) + " °C");
    
    // Report errors if any
    if (motor.hasErrors()) {
        uint16_t error_flags = motor.getLastFeedback().error_flags;
        Logger::error(motorLabel + " Error flags: 0x" + String(error_flags, HEX));
        Logger::error(motorLabel + " Error description: " + String(motor.getErrorText().c_str()));
    }
    
    Logger::info("--------------------------------");
}

bool MotorController::queryParameter(uint16_t index, float &value, uint8_t motor_id) {
    // Create a Type 17 frame to read the parameter
    uint32_t readId = ((0x11) << 24) | (MASTER_ID << 8) | motor_id;
    uint8_t readData[8] = {0};
    readData[0] = index & 0xFF;
    readData[1] = (index >> 8) & 0xFF;
    
    Logger::debug("Querying parameter 0x" + String(index, HEX) + " from motor ID " + String(motor_id));
    
    if (!CAN.beginExtendedPacket(readId & 0x1FFFFFFF)) {
        Logger::error("Failed to begin extended packet for motor ID " + String(motor_id));
        return false;
    }
    
    CAN.write(readData, 8);
    if (!CAN.endPacket()) {
        Logger::error("Failed to send packet for motor ID " + String(motor_id));
        return false;
    }
    
    // Wait for response
    unsigned long startTime = millis();
    while (millis() - startTime < 100) { // 100ms timeout
        int packetSize = CAN.parsePacket();
        if (packetSize) {
            uint32_t id = CAN.packetId();
            uint8_t responseType = (id >> 24) & 0x1F;
            uint8_t responseMotorId = (id >> 8) & 0xFF;
            
            uint8_t data[8] = {0};
            for (int i = 0; i < packetSize && i < 8; i++) {
                data[i] = CAN.read();
            }
            
            if (responseType == 0x11 && responseMotorId == motor_id) {
                uint16_t responseIndex = data[0] | (data[1] << 8);
                if (responseIndex == index) {
                    memcpy(&value, &data[4], sizeof(float));
                    Logger::debug("Got response for param 0x" + String(index, HEX) + " from motor ID " + 
                                String(motor_id) + ": " + String(value));
                    return true;
                }
            }
        }
        delay(1);
    }
    
    Logger::debug("Timeout waiting for parameter 0x" + String(index, HEX) + 
                  " from motor ID " + String(motor_id));
    
    return false;
}

void MotorController::setAllPixelsColor(int red, int green, int blue) {
    pixels.setPixelColor(0, pixels.Color(red, green, blue));
    pixels.show();
} 
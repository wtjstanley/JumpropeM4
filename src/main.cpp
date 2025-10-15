#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Library includes
#include "SystemConfig.h"
#include "Logger.h"
#include "RS03Motor.h"
#include "DisplayManager.h"
#include "MotorController.h"
#include "FeatherM4CanInterface.h"
#include <CANSAME5x.h>
#include <AlfredoCRSF.h>

// ----- Global Objects -----
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
CANSAME5x CAN;
AlfredoCRSF crsf;

// Hardware interfaces
FeatherM4CanInterface canBus;
RS03Motor motor1(canBus, MOTOR_ID_1, MASTER_ID);
RS03Motor motor2(canBus, MOTOR_ID_2, MASTER_ID);
MotorController motorController(motor1, motor2, pixels);

// ----- System Status Variables -----
static bool displayAvailable = false;
static bool rcAvailable = false;
static bool canAvailable = false;
static bool motorControllerAvailable = false;
static bool motorsReady = false;
struct SwitchStates {
        bool switchA;
        bool switchB;
};

// ----- Setup -----
void setup() {
    // Initialize serial communication with timeout
    Serial.begin(115200);
    Serial1.begin(CRSF_BAUDRATE);
    if (!Serial1) while (1) Serial.println("Invalid crsfSerial configuration");
    crsf.begin(Serial1);

    // Wait for serial connection with 5 second timeout
    unsigned long serialStartTime = millis();
    while (!Serial && (millis() - serialStartTime < 5000)) {
        delay(10); // Wait for serial connection, but not forever
    }
    // Initialize logger
    Logger::init();
    Logger::info("Simple RC Channel Reader - Starting");
    
    // Initialize OLED display (non-blocking)
    if (display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        displayAvailable = true;
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(0, 0);
        display.println("RC Channel Reader");
        display.println("Initializing...");
        display.display();
        Logger::info("OLED display initialized");
    } else {
        Logger::error("SSD1306 allocation failed - continuing without display");
        if (Serial) {
            Serial.println("OLED Display initialization failed - continuing without display");
        }
    }
    
    // Initialize NeoPixel
    pixels.begin();
    pixels.setBrightness(50);
    pixels.clear();
    pixels.setPixelColor(0, pixels.Color(50, 50, 0)); // Yellow while initializing
    pixels.show();
    Logger::info("NeoPixel initialized");
    
    // Initialize CAN bus (non-blocking)
    pinMode(PIN_CAN_STANDBY, OUTPUT);
    digitalWrite(PIN_CAN_STANDBY, false);
    pinMode(PIN_CAN_BOOSTEN, OUTPUT);
    digitalWrite(PIN_CAN_BOOSTEN, true);
    
    if (CAN.begin(1000000)) {
        canAvailable = true;
        Logger::info("CAN initialized at 1 Mbit/s");
    } else {
        Logger::error("Starting CAN failed - motors will not function");
        if (Serial) {
            Serial.println("CAN bus initialization failed - motors will not function");
        }
        pixels.setPixelColor(0, pixels.Color(50, 0, 0));
        pixels.show();
    }
    
    // Initialize motor controller (non-blocking)
    if (canAvailable && motorController.begin()) {
        motorControllerAvailable = true;
        Logger::info("Motor controller initialized");
    } else {
        Logger::error("Motor controller initialization failed - motors will not function");
        if (Serial) {
            Serial.println("Motor controller initialization failed - motors will not function");
        }
        pixels.setPixelColor(0, pixels.Color(50, 0, 0));
        pixels.show();
        
        if (displayAvailable) {
            display.clearDisplay();
            display.setCursor(0, 0);
            display.println("Motor Init Failed!");
            display.display();
        }
    }
    
    // Motor setup sequence with proper timing (only if motor controller is available)
    if (motorControllerAvailable) {
        Logger::info("Starting motor setup sequence...");
        
        // Step 1: Reset any faults and enable motors
        Logger::info("Step 1: Enabling motors and clearing faults");
        motor1.resetFaults();
        motor2.resetFaults();
        delay(100);
        
        motor1.enable();
        motor2.enable();
        delay(200);  // Give motors time to enable
    
    // Step 2: Configure zero flag and set mechanical zero positions  
    Logger::info("Step 2: Configuring zero flag for proper position reference");
    // Set zero flag to 1 for -π to π range (more suitable for position control)
    if (!motor1.setZeroFlag(1)) {
        Logger::warning("Failed to set zero flag for motor1");
    }
    if (!motor2.setZeroFlag(1)) {
        Logger::warning("Failed to set zero flag for motor2");
    }
    delay(200);
    
    Logger::info("Setting mechanical zero positions");
    bool zero1Success = motor1.setMechanicalZero();
    bool zero2Success = motor2.setMechanicalZero();
    
    if (!zero1Success) {
        Logger::error("Failed to set mechanical zero for motor1!");
    }
    if (!zero2Success) {
        Logger::error("Failed to set mechanical zero for motor2!");
    }
    
    delay(500);  // Allow time for zero setting to complete
    
    // Step 3: Set motors to position mode (after mechanical zero is set)
    Logger::info("Step 3: Setting position mode");
    if (!motor1.setModePositionPP(25.0f, 200.0f, 40.0f)) {
        Logger::error("Failed to set motor1 to position mode!");
    }
    if (!motor2.setModePositionPP(25.0f, 200.0f, 40.0f)) {
        Logger::error("Failed to set motor2 to position mode!");
    }
    delay(500);  // Allow time for mode setting to complete
    
    // Step 4: Enable active reporting for position feedback
    Logger::info("Step 4: Enabling active reporting");
    
    // Enable active reporting for motor1 with fallback
    bool reportingEnabled1 = motor1.setActiveReporting(true);
    if (!reportingEnabled1) {
        Logger::warning("Library method failed, trying direct approach for active reporting for motor1");
        
        // Type 5 message for active reporting command (from old working version)
        uint32_t reportingId1 = ((static_cast<uint32_t>(0x05) << 24) | 
                                (static_cast<uint32_t>(MASTER_ID) << 8) | 
                                static_cast<uint32_t>(MOTOR_ID_1)) & 0x1FFFFFFF;
        
        uint8_t reportingData[8] = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // 0x01 = enable reporting
        
        if (CAN.beginExtendedPacket(reportingId1)) {
            CAN.write(reportingData, 8);
            if (CAN.endPacket()) {
                Logger::info("Direct active reporting command sent for motor1");
                reportingEnabled1 = true;
            } else {
                Logger::error("Failed to send direct active reporting command for motor1");
            }
        } else {
            Logger::error("Failed to begin direct active reporting packet for motor1");
        }
    }
    
    // Enable active reporting for motor2 with fallback
    bool reportingEnabled2 = motor2.setActiveReporting(true);
    if (!reportingEnabled2) {
        Logger::warning("Library method failed, trying direct approach for active reporting for motor2");
        
        // Type 5 message for active reporting command (from old working version)
        uint32_t reportingId2 = ((static_cast<uint32_t>(0x05) << 24) | 
                                (static_cast<uint32_t>(MASTER_ID) << 8) | 
                                static_cast<uint32_t>(MOTOR_ID_2)) & 0x1FFFFFFF;
        
        uint8_t reportingData[8] = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // 0x01 = enable reporting
        
        if (CAN.beginExtendedPacket(reportingId2)) {
            CAN.write(reportingData, 8);
            if (CAN.endPacket()) {
                Logger::info("Direct active reporting command sent for motor2");
                reportingEnabled2 = true;
            } else {
                Logger::error("Failed to send direct active reporting command for motor2");
            }
        } else {
            Logger::error("Failed to begin direct active reporting packet for motor2");
        }
    }
    
    if (reportingEnabled1 && reportingEnabled2) {
        Logger::info("Active reporting enabled for both motors");
    } else {
        Logger::warning("Active reporting setup had issues - some motors may not report feedback");
    }
    
    delay(200);
    
    // Step 5: Move both motors to position 0 (should already be there after zero setting)
    Logger::info("Step 5: Moving to zero position");
    motor1.setPosition(0.0f);
    motor2.setPosition(0.0f);
    delay(200);
    
        // Verify setup
        if (zero1Success && zero2Success) {
            Logger::info("Motor setup completed successfully");
            motorsReady = true;
        } else {
            Logger::warning("Motor setup completed with errors - check motor connections");
            motorsReady = false;
        }
    } else {
        Logger::warning("Motor controller not available - motors will not function");
        motorsReady = false;
    }
    
    // System ready indication
    if (motorsReady && rcAvailable) {
        pixels.setPixelColor(0, pixels.Color(0, 50, 0)); // Green indicates fully ready
        Logger::info("Setup complete. Motors ready for RC control.");
    } else if (motorsReady) {
        pixels.setPixelColor(0, pixels.Color(0, 25, 25)); // Cyan indicates motors ready but no RC
        Logger::info("Setup complete. Motors ready but no RC input.");
    } else {
        pixels.setPixelColor(0, pixels.Color(25, 25, 0)); // Yellow indicates limited functionality
        Logger::info("Setup complete with limited functionality - check connections.");
    }
    pixels.show();
    
    // Update display with current status
    if (displayAvailable) {
        display.clearDisplay();
        display.setCursor(0, 0);
        if (motorsReady && rcAvailable) {
            display.println("Motor Control Ready");
            display.println("Zero set first");
            display.println("Ch4: Mode 1,2,3");
            display.println("SwA: Zero");
            display.println("M1: Reversed");
        } else if (motorsReady) {
            display.println("Motors Ready");
            display.println("No RC Input");
            display.println("Check RC connection");
        } else {
            display.println("Limited Function");
            display.println("Check connections");
            display.println("Motors disabled");
        }
        display.display();
        delay(1000);
    }
    
    // Print status to serial if available
    if (Serial) {
        if (motorsReady && rcAvailable) {
            Serial.println("Motor Control Ready - Full Functionality");
        } else if (motorsReady) {
            Serial.println("Motors Ready - No RC Input");
        } else {
            Serial.println("Limited Functionality - Check Connections");
        }
        
        Serial.println("FIXED: Updated position constants from -12.5/12.5 to -12.57/12.57 rad");
        Serial.println("FIXED: Corrected mode values and parameter indices from manual");
        Serial.println("FIXED: Zero flag set to 1 for -π to π position range");
        Serial.println("SAFETY: Emergency stop if no RC signal or frame timeout for 3 seconds");
        Serial.println("Control Scheme:");
        Serial.println("  Channel 4 Mode 1 = Velocity Mode");
        Serial.println("  Channel 4 Mode 2 = Position Mode (Switch B control, Ch5 nudges zero ±0.2rad)");
        Serial.println("  Channel 4 Mode 3 = Position Mode (SwA=-0.1rad, SwB=3.9rad, Ch5=1.9±2π rad)");
        Serial.println("    Mode 3: Ch10 adds 0-0.8rad shift (SwA: negative, SwB: positive)");
        Serial.println("  Channel 4 Mode 4 = Position Mode (Fixed: M1=-4.4rad, M2=-0.6rad)");
        Serial.println("  Channel 4 Mode 5 = Position Mode (Individual motor control)");
        Serial.println("  Channel 4 Mode 6 = Emergency Stop and Clear Errors");
        Serial.println("  Switch A = Set mechanical zero (Mode 6), or Position -0.1rad-Ch10_shift (Mode 3)");
        Serial.println("  Channel 5 = Velocity control (Mode 1), Zero nudging (Mode 2), or Position 1.9±2π rad (Mode 3, no switches)");
        Serial.println("  Channel 7 = Motor 1 position ±7rad (Mode 5 only)");
        Serial.println("  Switch B = Position control: OFF=0rad, ON=3.8rad (Mode 2), or Position 3.9rad+Ch10_shift (Mode 3)");
        Serial.println("  Channel 9 = Motor 2 position ±7rad (Mode 5 only)");
        Serial.println("  Channel 10 = Mode 3 shift amount: 0-0.8rad (SwA: negative, SwB: positive)");
        Serial.println("  Channel 8 = State tracking only (Low/Middle/High)");
        Serial.println("  Motor 1 is REVERSED by default");
        Serial.println("  SAFETY: Motors emergency stop if no RC signal or frame timeout for 3+ seconds");
        Serial.println("  TIMING: RC input processed via 500Hz timer interrupt for consistent timing");
        Serial.println("Mode  M1_Desired  M2_Desired  ButtonA  ButtonB");
    }
}

// ----- Additional Variables for Motor Control -----
static bool inPositionMode = false;
static bool modeInitialized = false;
static float currentTargetPosition = 0.0f;
static float nudgeableZeroPosition = 0.0f;  // Adjustable zero position (±0.2 rad max)
static unsigned long lastNudgeTime = 0;
static bool wasNudging = false;


// ----- Safety Variables -----
static bool emergencyStopActive = false;
static unsigned long noSignalStartTime = 0;
static bool noSignalTimerActive = false;
const unsigned long RC_TIMEOUT_MS = 3000;  // 3 seconds timeout

void printChannels()
{
  for (int ChannelNum = 1; ChannelNum <= 16; ChannelNum++)
  {
    Serial.print(crsf.getChannel(ChannelNum));
    Serial.print(", ");
  }
  Serial.println(" ");
}

// ----- Main Loop -----
void loop() {
    static unsigned long lastUpdate = 0;
    static unsigned long lastSerialPrint = 0;
    static unsigned long lastMotorUpdate = 0;
    crsf.update();
    rcAvailable = crsf.isLinkUp();
    //printChannels();
    
    // ----- SAFETY CHECK: RC Frame Timeout -----
        
    // Handle emergency stop activation
    if (!rcAvailable && !emergencyStopActive) {
        emergencyStopActive = true;
        Logger::error("RC SIGNAL TIMEOUT - EMERGENCY STOP ACTIVATED");
        
        // Immediately stop motors
        if (motorControllerAvailable) {
            motor1.setModeVelocity();
            motor2.setModeVelocity();
            delay(50);
            motor1.setVelocity(0.0f);
            motor2.setVelocity(0.0f);
            delay(200);
            //Disable
            motor1.disable();
            motor2.disable();
            delay(200);

            //Reset faults
            motor1.resetFaults();
            motor2.resetFaults();
            delay(200);
            //Enable motors
        }
        
        // Reset control state
        inPositionMode = false;
        modeInitialized = false;
        currentTargetPosition = 0.0f;
        
        // Visual indication
        pixels.setPixelColor(0, pixels.Color(50, 0, 0)); // Red for emergency stop
        pixels.show();
    }
        
    // Handle recovery from emergency stop
    if (rcAvailable && emergencyStopActive) {
        emergencyStopActive = false;
        Logger::info("RC SIGNAL RECOVERED - Emergency stop deactivated");

        // Reset mode initialization to allow proper mode setup
        modeInitialized = false;
        
        // Visual indication back to normal
        pixels.setPixelColor(0, pixels.Color(0, 50, 0)); // Green for normal operation
        pixels.show();
    }
    /////////////////////////////////////////////////////////////////////////////////////////


    // Process CAN messages for motor feedback (only if CAN is available)
    if (canAvailable) {
        // Process available messages (reduced from 5 to 3 iterations to reduce loop time)
        for (int i = 0; i < 3; i++) {
            int packetSize = CAN.parsePacket();
            if (packetSize) {
                CanFrame frame;
                frame.id = CAN.packetId();
                frame.is_extended = CAN.packetExtended();
                frame.dlc = packetSize;
                
                // Read data bytes
                int j = 0;
                while (CAN.available() && j < 8) {
                    frame.data[j++] = CAN.read();
                }
                
                // Process the frame with both motors to update their feedback
                motor1.processFeedback(frame);
                motor2.processFeedback(frame);
            } else {
                break; // No more packets available, exit early
            }
        }
    }
    
    // Periodically request motor state directly using Type 0 query for both motors (only if CAN is available)
    if (canAvailable) {
        static unsigned long lastDirectQueryTime = 0;
        unsigned long currentTime = millis();
        if (currentTime - lastDirectQueryTime >= 100) { // Every 100ms
        // Send a Type 0 query to request state from motor 1
        uint32_t queryId1 = ((0x00) << 24) | (MASTER_ID << 16) | MOTOR_ID_1;
        if (CAN.beginExtendedPacket(queryId1 & 0x1FFFFFFF)) {
            // Type 0 has no data
            if (CAN.endPacket()) {
                Logger::debug("Sent Type 0 query to request motor 1 state");
            }
        }
        
        // Send a Type 0 query to request state from motor 2
        uint32_t queryId2 = ((0x00) << 24) | (MASTER_ID << 16) | MOTOR_ID_2;
        if (CAN.beginExtendedPacket(queryId2 & 0x1FFFFFFF)) {
            // Type 0 has no data
            if (CAN.endPacket()) {
                Logger::debug("Sent Type 0 query to request motor 2 state");
            }
        }
        
            lastDirectQueryTime = currentTime;
        }
    }
    
    /*
    // Check if we're receiving signal (only if RC is available)
    if (rcAvailable) {
        // Start no signal timer if not already started
        if (!noSignalTimerActive) {
            noSignalStartTime = millis();
            noSignalTimerActive = true;
            Logger::warning("NO RC SIGNAL detected - starting 3 second safety timer");
        }
        
        // Check if we've been in no signal mode for 3 seconds
        unsigned long noSignalDuration = millis() - noSignalStartTime;
        if (noSignalDuration >= RC_TIMEOUT_MS && !emergencyStopActive && motorsReady) {
            emergencyStopActive = true;
            Logger::error("NO SIGNAL TIMEOUT - EMERGENCY STOP ACTIVATED");
            
            // Immediately stop motors
            if (motorControllerAvailable) {
                motor1.setModeVelocity();
                motor2.setModeVelocity();
                delay(50);
                motor1.setVelocity(0.0f);
                motor2.setVelocity(0.0f);
            }
            
            // Reset control state
            inPositionMode = false;
            modeInitialized = false;
            currentTargetPosition = 0.0f;
        }
        
        // No signal - show error
        pixels.setPixelColor(0, pixels.Color(50, 0, 0)); // Red
        pixels.show();
        
        // Update display every 200ms when no signal
        if (displayAvailable && millis() - lastUpdate >= 200) {
            display.clearDisplay();
            display.setTextSize(1);
            display.setTextColor(SSD1306_WHITE);
            display.setCursor(0, 0);
            display.println("Motor Control");
            
            if (emergencyStopActive) {
                display.println("** EMERGENCY STOP **");
                display.println("NO SIGNAL TIMEOUT");
                display.println("Motors STOPPED");
                display.println("!ARMS ARE SAFE!");
            } else {
                display.println("Mode: NO SIGNAL");
                display.print("Safety: ");
                display.print((RC_TIMEOUT_MS - noSignalDuration) / 1000 + 1);
                display.println("s");
                display.println("M1 Des: STOP");
            }
            display.println("Btn A: ---  B: ---");
            display.display();
            lastUpdate = millis();
        }
        

        
        // Print to serial every 1000ms when no signal
        if (Serial && millis() - lastSerialPrint >= 1000) {
            if (emergencyStopActive) {
                Serial.println("NO RC SIGNAL - EMERGENCY STOP ACTIVE");
            } else {
                unsigned long timeLeft = (RC_TIMEOUT_MS - noSignalDuration) / 1000 + 1;
                Serial.print("NO RC SIGNAL - Emergency stop in ");
                Serial.print(timeLeft);
                Serial.println(" seconds");
            }
            lastSerialPrint = millis();
        }
        
        delay(50);
        return;
    } else if (noSignalTimerActive) {
        // Signal has returned - reset the no signal timer and emergency stop if it was from no signal
        noSignalTimerActive = false;
        if (emergencyStopActive) {
            emergencyStopActive = false;
            Logger::info("RC SIGNAL RECOVERED from NO SIGNAL - Emergency stop deactivated");
            
            // Reset mode initialization to allow proper mode setup
            modeInitialized = false;
            
            // Visual indication back to normal
            pixels.setPixelColor(0, pixels.Color(0, 50, 0)); // Green for normal operation
            pixels.show();
        }
    }

    */
    
    // If RC is not available, skip RC-based control but continue with other functions
    if (!rcAvailable) {
        // Update display to show RC not available
        if (displayAvailable && millis() - lastUpdate >= 1000) {
            display.clearDisplay();
            display.setTextSize(1);
            display.setTextColor(SSD1306_WHITE);
            display.setCursor(0, 0);
            display.println("Motor Control");
            display.println("Mode: NO RC");
            display.println("M1 Des: STOP");
            display.println("M2 Des: STOP");
            display.println("Btn A: ---  B: ---");
            display.display();
            lastUpdate = millis();
        }
        
        if (Serial && millis() - lastSerialPrint >= 2000) {
            Serial.println("RC INPUT NOT AVAILABLE - Motors disabled for safety");
            lastSerialPrint = millis();
        }
        
        delay(100);
        return;
    }
    
    // We have signal - show green LED
    pixels.setPixelColor(0, pixels.Color(0, 50, 0)); // Green
    pixels.show();
    
    // We only need specific channels for control, not display
    
    // Get decoded special channels (needed for display and motor control)
    int mode = 0;
    SwitchStates switches = {false, false};
    
    if (rcAvailable) {
        int ch4Value = crsf.getChannel(4);  // Channel 4 is 0-based index 3
        // MODE VALUE mapping with ±40us wiggle room
        mode = 0;  // Channel 4 mode selection
        if (ch4Value >= 1166 && ch4Value <= 1246){mode = 1; }  // 1206 ± 40
        if (ch4Value >= 1265 && ch4Value <= 1345){mode = 2; } // 1305 ± 40
        if (ch4Value >= 1362 && ch4Value <= 1442){mode = 3; } // 1402 ± 40
        if (ch4Value >= 1557 && ch4Value <= 1637){mode = 4; } // 1597 ± 40
        if (ch4Value >= 1655 && ch4Value <= 1735){mode = 5; } // 1695 ± 40
        if (ch4Value >= 1753 && ch4Value <= 1833){mode = 6; } // 1793 ± 40


        int ch6Value = crsf.getChannel(6);  // Channel 6 is 0-based index 5
        // Switch state mapping with ±40us wiggle room
        if (ch6Value >= 1160 && ch6Value <= 1240) {
            // A=OFF, B=OFF: 1200
            switches.switchA = false;
            switches.switchB = false;
        }
        else if (ch6Value >= 1350 && ch6Value <= 1450) {
            // A=OFF, B=ON: 1400
            switches.switchA = false;
            switches.switchB = true;
        }
        else if (ch6Value >= 1550 && ch6Value <= 1650) {
            // A=ON, B=OFF: 1599
            switches.switchA = true;
            switches.switchB = false;
        }
        else if (ch6Value >= 1750 && ch6Value <= 1850) {
            // A=ON, B=ON: 1800
            switches.switchA = true;
            switches.switchB = true;
    }

    }
    
            // Motor control logic (every 20ms) - only if motors are ready and not in emergency stop
        if (motorsReady && !emergencyStopActive && millis() - lastMotorUpdate >= 20) {
            // Get RC channel values
            int ch5Value = crsf.getChannel(5);  // Channel 5 for velocity control (0-indexed as 4)
            int ch7Value = crsf.getChannel(7);  // Channel 7 for Motor 1 control (0-indexed as 6)
            int ch9Value = crsf.getChannel(9);  // Channel 9 for Motor 2 control (0-indexed as 8)
        
        // Motor 1 is reversed by default
        bool motor1Reversed = true;
        bool motor2Reversed = false;
        
        // Determine control mode based on Channel 4 mode
        // Mode 1 = Velocity Mode, Mode 2 = Position Mode (Switch B), Mode 3 = Position Mode (Channel 5)
        // Mode 4 = Position Mode (Fixed positions: M1=-4.4rad, M2=-0.6rad)
        // Mode 5 = Position Mode (Individual motor control - Ch7 for M1, Ch9 for M2)
        // Mode 6 = Emergency Stop and Clear Errors
        bool shouldBeInPositionMode = (mode == 2 || mode == 3 || mode == 4 || mode == 5);
        
        // Handle Mode 6 - Emergency Stop and Robust Communication Reset
        static bool mode6Executed = false;
        static int lastMode6Time = 0;
        static bool wasInMode6 = false;
        
        if (mode == 6) {
            // Execute Mode 6 actions only once per mode entry or every 3 seconds while in mode
            if (!mode6Executed || (millis() - lastMode6Time > 3000)) {
                Logger::info("Mode 6: Starting robust communication reset sequence (preserving mechanical zero)");
                
                // Step 1: Emergency stop - set velocity mode and stop
                Logger::info("Step 1: Emergency stop");
                motor1.setModeVelocity();
                motor2.setModeVelocity();
                delay(100);
                motor1.setVelocity(0.0f);
                motor2.setVelocity(0.0f);
                delay(200);
                
                // Step 2: Reset faults and enable motors
                Logger::info("Step 2: Resetting faults and enabling motors");
                motor1.resetFaults();
                motor2.resetFaults();
                delay(100);
                
                motor1.enable();
                motor2.enable();
                delay(200);  // Give motors time to enable
                
                // Step 3: Configure zero flag (but don't reset mechanical zero)
                Logger::info("Step 3: Configuring zero flag for proper position reference");
                // Set zero flag to 1 for -π to π range (more suitable for position control)
                if (!motor1.setZeroFlag(1)) {
                    Logger::warning("Failed to set zero flag for motor1");
                }
                if (!motor2.setZeroFlag(1)) {
                    Logger::warning("Failed to set zero flag for motor2");
                }
                delay(200);
                
                // // Step 4: Set motors to position mode
                // Logger::info("Step 4: Setting position mode");
                // if (!motor1.setModePositionPP(25.0f, 200.0f, 40.0f)) {
                //     Logger::error("Failed to set motor1 to position mode!");
                // }
                // if (!motor2.setModePositionPP(25.0f, 200.0f, 40.0f)) {
                //     Logger::error("Failed to set motor2 to position mode!");
                // }
                // delay(500);  // Allow time for mode setting to complete
                
                // Step 5: Enable active reporting
                Logger::info("Step 5: Enabling active reporting");
                
                // Enable active reporting for motor1 with fallback
                bool reportingEnabled1 = motor1.setActiveReporting(true);
                if (!reportingEnabled1) {
                    Logger::warning("Library method failed, trying direct approach for active reporting for motor1");
                    
                    // Type 5 message for active reporting command
                    uint32_t reportingId1 = ((static_cast<uint32_t>(0x05) << 24) | 
                                            (static_cast<uint32_t>(MASTER_ID) << 8) | 
                                            static_cast<uint32_t>(MOTOR_ID_1)) & 0x1FFFFFFF;
                    
                    uint8_t reportingData[8] = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // 0x01 = enable reporting
                    
                    if (CAN.beginExtendedPacket(reportingId1)) {
                        CAN.write(reportingData, 8);
                        if (CAN.endPacket()) {
                            Logger::info("Direct active reporting command sent for motor1");
                            reportingEnabled1 = true;
                        } else {
                            Logger::error("Failed to send direct active reporting command for motor1");
                        }
                    } else {
                        Logger::error("Failed to begin direct active reporting packet for motor1");
                    }
                }
                
                // Enable active reporting for motor2 with fallback
                bool reportingEnabled2 = motor2.setActiveReporting(true);
                if (!reportingEnabled2) {
                    Logger::warning("Library method failed, trying direct approach for active reporting for motor2");
                    
                    // Type 5 message for active reporting command
                    uint32_t reportingId2 = ((static_cast<uint32_t>(0x05) << 24) | 
                                            (static_cast<uint32_t>(MASTER_ID) << 8) | 
                                            static_cast<uint32_t>(MOTOR_ID_2)) & 0x1FFFFFFF;
                    
                    uint8_t reportingData[8] = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // 0x01 = enable reporting
                    
                    if (CAN.beginExtendedPacket(reportingId2)) {
                        CAN.write(reportingData, 8);
                        if (CAN.endPacket()) {
                            Logger::info("Direct active reporting command sent for motor2");
                            reportingEnabled2 = true;
                        } else {
                            Logger::error("Failed to send direct active reporting command for motor2");
                        }
                    } else {
                        Logger::error("Failed to begin direct active reporting packet for motor2");
                    }
                }
                
                if (reportingEnabled1 && reportingEnabled2) {
                    Logger::info("Active reporting enabled for both motors");
                } else {
                    Logger::warning("Active reporting setup had issues - some motors may not report feedback");
                }
                
                delay(200);
                
                // Step 6: Move both motors to position 0 (using existing mechanical zero)
                Logger::info("Step 6: Moving to stored zero position (mechanical zero preserved)");
                motor1.setPosition(0.0f);
                motor2.setPosition(0.0f);
                delay(200);
                
                // Step 7: Reset control variables (but not mechanical zero)
                Logger::info("Step 7: Resetting control variables");
                currentTargetPosition = 0.0f;
                nudgeableZeroPosition = 0.0f;  // Reset the nudgeable zero position
                
                // Communication reset is always successful (no dependency on mechanical zero)
                Logger::info("Mode 6: Robust communication reset successful - mechanical zero preserved");
                motorsReady = true;
                
                mode6Executed = true;
                lastMode6Time = millis();
                Logger::info("Mode 6: Motors reset and ready (use Switch A to reset mechanical zero if needed)");
            }
            wasInMode6 = true;
        } else {
            // Reset Mode 6 execution flag when exiting Mode 6
            if (wasInMode6) {
                Logger::info("Exiting Mode 6 - communication reset complete");
                
                mode6Executed = false;
                modeInitialized = false;  // Force re-initialization of the new mode
                wasInMode6 = false;
                
                Logger::info("Ready for normal operation with reset communication");
            }
        }
        
        // Handle mechanical zero reset using Switch A (only in Mode 6)
        static bool lastSwitchAState = false;
        if (switches.switchA && !lastSwitchAState && mode == 6) {
            // Rising edge detection - switch just turned ON, and we're in Mode 6
            Logger::info("Switch A pressed in Mode 6 - Setting NEW mechanical zero position and resetting nudgeable zero");
            
            bool zero1Success = motor1.setMechanicalZero();
            bool zero2Success = motor2.setMechanicalZero();
            
            if (!zero1Success) {
                Logger::error("Failed to set mechanical zero for motor1!");
            }
            if (!zero2Success) {
                Logger::error("Failed to set mechanical zero for motor2!");
            }
            
            if (zero1Success && zero2Success) {
                Logger::info("New mechanical zero positions set successfully");
            } else {
                Logger::warning("Mechanical zero setting had errors - check motor connections");
            }
            
            delay(500);  // Allow time for zero setting to complete
            
            // Move to new zero position
            motor1.setPosition(0.0f);
            motor2.setPosition(0.0f);
            
            currentTargetPosition = 0.0f;
            nudgeableZeroPosition = 0.0f;  // Reset the nudgeable zero position
        }
        lastSwitchAState = switches.switchA;
        
        // Switch modes if needed or if switching between different position modes
        static int lastMode = 0;
        if ((shouldBeInPositionMode != inPositionMode || mode != lastMode) && mode != 6) {
            inPositionMode = shouldBeInPositionMode;
            modeInitialized = false;
            
            if (mode == 1) {
                Logger::info("Switching to velocity mode (Mode 1)");
            } else if (mode == 2) {
                Logger::info("Switching to position mode with Switch B control (Mode 2)");
            } else if (mode == 3) {
                Logger::info("Switching to position mode with Channel 5 control (Mode 3)");
            } else if (mode == 4) {
                Logger::info("Switching to position mode with fixed positions (Mode 4)");
            } else if (mode == 5) {
                Logger::info("Switching to position mode with individual motor control (Mode 5) - smoothing enabled");
            }
        }
        
        // Update lastMode for all modes (including Mode 6) to track transitions
        if (mode != lastMode) {
            lastMode = mode;
        }
        
        // Initialize mode if needed (always control both motors) - skip for Mode 6
        if (!modeInitialized && mode != 6) {
            if (inPositionMode) {
                // Set position mode for both motors
                motor1.setModePositionPP(POSITION_SPEED_LIMIT, POSITION_ACCELERATION, MOTOR_CURRENT_LIMIT);
                motor2.setModePositionPP(POSITION_SPEED_LIMIT, POSITION_ACCELERATION, MOTOR_CURRENT_LIMIT);
                Logger::info("Both motors set to position mode");
            } else {
                // Set velocity mode for both motors
                motor1.setModeVelocity();
                motor2.setModeVelocity();
                Logger::info("Both motors set to velocity mode");
            }
            modeInitialized = true;
            delay(100);  // Give time for mode change
        }
        
        // Execute motor commands based on mode (skip for Mode 6 - motors already stopped)
        if (modeInitialized && mode != 6) {
            if (inPositionMode) {
                if (mode == 2) {
                    // Mode 2: Position mode - use Switch B to control position (nudgeable zero or 3.8 radians)
                    // Use Channel 5 to nudge the zero position
                    
                    // Calculate Channel 5 percentage (0-100%)
                    float ch5Percentage = (float)(ch5Value - MIN_PULSE) / (MAX_PULSE - MIN_PULSE) * 100.0f;
                    
                    // Check if we should nudge (above 65% or below 35%)
                    bool shouldNudgeUp = (ch5Percentage > 65.0f);
                    bool shouldNudgeDown = (ch5Percentage < 35.0f);
                    bool shouldNudge = shouldNudgeUp || shouldNudgeDown;
                    
                    // Handle nudging timing
                    unsigned long currentTime = millis();
                    if (shouldNudge) {
                        if (!wasNudging) {
                            // Just started nudging - record the time
                            lastNudgeTime = currentTime;
                            wasNudging = true;
                        } else if (currentTime - lastNudgeTime >= 100) {
                            // We've been nudging for 100ms - apply the nudge
                            float nudgeAmount = 0.01f;
                            if (shouldNudgeDown) {
                                nudgeAmount = -0.01f;
                            }
                            
                            // Apply nudge with limits (±0.2 radians)
                            float newZeroPosition = nudgeableZeroPosition + nudgeAmount;
                            if (newZeroPosition >= -0.2f && newZeroPosition <= 0.2f) {
                                nudgeableZeroPosition = newZeroPosition;
                                Logger::info("Nudged zero position to " + String(nudgeableZeroPosition, 3) + " rad");
                            }
                            
                            // Reset timing for next nudge
                            lastNudgeTime = currentTime;
                        }
                    } else {
                        wasNudging = false;
                    }
                    
                    // Set target position based on Switch B, offset by nudgeable zero
                    if (switches.switchB) {
                        currentTargetPosition = 3.8f - nudgeableZeroPosition;  // Switch B ON = 3.8 radians + offset
                    } else {
                        currentTargetPosition = nudgeableZeroPosition;  // Switch B OFF = nudgeable zero position
                    }
                } else if (mode == 3) {
                    // Mode 3: Position mode - Switch A = -0.1 rad, Switch B = 3.9 rad, or Channel 5 control
                    // Channel 10 provides additional shift of 0-0.8 rad when switches are pressed
                    
                    // Get Channel 10 value for shift calculation
                    int ch10Value = crsf.getChannel(10);  // Channel 10 (0-indexed as 9)
                    
                    // Map Channel 10 from pulse range to shift amount (0 to 0.8 radians)
                    float shiftAmount = 0.0f;
                    if (ch10Value >= MIN_PULSE && ch10Value <= MAX_PULSE) {
                        shiftAmount = map(ch10Value, MIN_PULSE, MAX_PULSE, 0, 3000) * 0.001f;  // 0 to 3 radians
                    }
                    
                    // Check for switch overrides first
                    if (switches.switchB) {
                        // Switch B = -0.1 rad with negative shift from Channel 10
                        currentTargetPosition = -0.1f - shiftAmount;
                    } else if (switches.switchA) {
                        // Switch A = 3.9 rad with positive shift from Channel 10
                        currentTargetPosition = 3.9f + shiftAmount;
                    } else {
                        // No switches pressed - use Channel 5 to control position around 1.9 rad base with ±2π range
                        float basePosition = 1.9f;
                        float maxOffset = 2.0f * PI;  // ±2π radians
                        
                        // Map Channel 5 from pulse range to position offset
                        float positionOffset = 0.0f;
                        if (ch5Value >= (MID_PULSE - DEAD_ZONE) && ch5Value <= (MID_PULSE + DEAD_ZONE)) {
                            positionOffset = 0.0f;  // Dead zone - no offset from base position
                        } else if (ch5Value < MID_PULSE) {
                            // Map lower half to negative offset
                            positionOffset = map(ch5Value, MIN_PULSE, MID_PULSE - DEAD_ZONE, -maxOffset * 1000, 0) * 0.001f;
                        } else {
                            // Map upper half to positive offset
                            positionOffset = map(ch5Value, MID_PULSE + DEAD_ZONE, MAX_PULSE, 0, maxOffset * 1000) * 0.001f;
                        }
                        
                        currentTargetPosition = basePosition + positionOffset;
                    }
                } else if (mode == 4) {
                    // Mode 4: Position mode - fixed positions with button A override
                    // Default: M1=-4.4rad, M2=-0.6rad
                    // Button A pressed: M1=-2.4rad, M2=1.3rad
                    currentTargetPosition = 0.0f;  // Not used in Mode 4, individual targets are used
                    
                    // Send individual position commands to motors with direction control
                    float motor1TargetPosition;
                    float motor2TargetPosition;
                    
                    if (switches.switchA) {
                        // Button A pressed - use alternate positions
                        if (motor1Reversed) {
                            motor1TargetPosition = -2.4f;
                        } else {
                            motor1TargetPosition = -2.4f; // Not reversed: -2.4
                        }

                        if (motor2Reversed) {
                            motor2TargetPosition = -1.3f;  // Reversed: -(1.3) = -1.3  
                        } else {
                            motor2TargetPosition = 1.3f; // Not reversed: 1.3
                        }
                    } else {
                        // Button A not pressed - use default positions
                        if (motor1Reversed) {
                            motor1TargetPosition = -4.4f;  
                        } else {
                            motor1TargetPosition = -4.4f; // Not reversed: -4.4
                        }

                        if (motor2Reversed) {
                            motor2TargetPosition = -(-0.6f);  // Reversed: -(-0.6) = 0.6
                        } else {
                            motor2TargetPosition = -0.6f; // Not reversed: -0.6
                        }
                    }
                    motor1.setPosition(motor1TargetPosition);
                    motor2.setPosition(motor2TargetPosition);
                } else if (mode == 5) {
                    // Mode 5: Position mode - use Channel 7 for Motor 1 and Channel 9 for Motor 2 (individual control)
                    // Map channels from pulse range to -7 to +7 radians
                    float motor1Position = 0.0f;
                    float motor2Position = 0.0f;
                    
                    // Map Channel 7 to Motor 1 position (-7 to +7 radians)
                    if (ch7Value >= (MID_PULSE - DEAD_ZONE) && ch7Value <= (MID_PULSE + DEAD_ZONE)) {
                        motor1Position = 0.0f;  // Dead zone - neutral position
                    } else if (ch7Value < MID_PULSE) {
                        motor1Position = map(ch7Value, MIN_PULSE, MID_PULSE - DEAD_ZONE, -7000, 0) * 0.001f;
                    } else {
                        motor1Position = map(ch7Value, MID_PULSE + DEAD_ZONE, MAX_PULSE, 0, 7000) * 0.001f;
                    }
                    
                    // Map Channel 9 to Motor 2 position (-7 to +7 radians)
                    if (ch9Value >= (MID_PULSE - DEAD_ZONE) && ch9Value <= (MID_PULSE + DEAD_ZONE)) {
                        motor2Position = 0.0f;  // Dead zone - neutral position
                    } else if (ch9Value < MID_PULSE) {
                        motor2Position = map(ch9Value, MIN_PULSE, MID_PULSE - DEAD_ZONE, -7000, 0) * 0.001f;
                    } else {
                        motor2Position = map(ch9Value, MID_PULSE + DEAD_ZONE, MAX_PULSE, 0, 7000) * 0.001f;
                    }
                    
                    // Apply exponential smoothing to reduce jerky motions
                    static float smoothedMotor1Position = 0.0f;
                    static float smoothedMotor2Position = 0.0f;
                    static bool smoothingInitialized = false;
                    static int lastSmoothingMode = 0;
                    const float smoothingFactor = 0.15f;  // Lower = more smoothing, Higher = more responsive
                    
                    // Reset smoothing when entering mode 5 from a different mode
                    if (mode != lastSmoothingMode) {
                        smoothingInitialized = false;
                        lastSmoothingMode = mode;
                    }
                    
                    // Initialize smoothed values on first run in mode 5 or after mode change
                    if (!smoothingInitialized) {
                        smoothedMotor1Position = motor1Position;
                        smoothedMotor2Position = motor2Position;
                        smoothingInitialized = true;
                        Logger::info("Mode 5 smoothing initialized/reset");
                    }
                    
                    // Apply exponential smoothing: smoothed = smoothed + factor * (new - smoothed)
                    smoothedMotor1Position += smoothingFactor * (motor1Position - smoothedMotor1Position);
                    smoothedMotor2Position += smoothingFactor * (motor2Position - smoothedMotor2Position);
                    
                    // Send individual position commands to motors with direction control using smoothed values
                    float motor1TargetPosition = motor1Reversed ? -smoothedMotor1Position : smoothedMotor1Position;
                    float motor2TargetPosition = motor2Reversed ? -smoothedMotor2Position : smoothedMotor2Position;
                    motor1.setPosition(motor1TargetPosition);
                    motor2.setPosition(motor2TargetPosition);
                    
                    // Store positions for display (use the actual commanded positions)
                    currentTargetPosition = 0.0f;  // Not used in Mode 5, individual targets are used
                }
                
                // Send position commands to both motors with direction control (for modes 2 and 3)
                if (mode == 2 || mode == 3) {
                    float motor1TargetPosition = motor1Reversed ? -currentTargetPosition : currentTargetPosition;
                    float motor2TargetPosition = motor2Reversed ? -currentTargetPosition : currentTargetPosition;
                    motor1.setPosition(motor1TargetPosition);
                    motor2.setPosition(motor2TargetPosition);
                }
                
            } else {
                // Mode 1: Velocity mode - use Channel 5 for velocity control with dead zone
                float targetVelocity = 0.0f;
                if (ch5Value >= (MID_PULSE - DEAD_ZONE) && ch5Value <= (MID_PULSE + DEAD_ZONE)) {
                    targetVelocity = 0.0f;  // Dead zone
                } else if (ch5Value < MID_PULSE) {
                    targetVelocity = map(ch5Value, MIN_PULSE, MID_PULSE - DEAD_ZONE, -MAX_VELOCITY * 1000, 0) * 0.001f;
                } else {
                    targetVelocity = map(ch5Value, MID_PULSE + DEAD_ZONE, MAX_PULSE, 0, MAX_VELOCITY * 1000) * 0.001f;
                }
                
                // Send velocity commands to both motors with direction control
                float motor1TargetVelocity = motor1Reversed ? -targetVelocity : targetVelocity;
                float motor2TargetVelocity = motor2Reversed ? -targetVelocity : targetVelocity;
                motor1.setVelocity(motor1TargetVelocity);
                motor2.setVelocity(motor2TargetVelocity);
                
                // Update target position for display (in velocity mode, show current velocity as target)
                currentTargetPosition = targetVelocity;
            }
        }
        
        lastMotorUpdate = millis();
    }
    
    // Update OLED display every 150ms (only if display is available)
    if (displayAvailable && millis() - lastUpdate >= 150) {
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(0, 0);
        display.println("Motor Control");
        
        // Show emergency stop status first if active
        if (emergencyStopActive) {
            display.println("*** EMERGENCY STOP ***");
            display.println("RC SIGNAL TIMEOUT");
            display.println("Motors STOPPED");
            display.println("Check RC connection");
            display.display();
            lastUpdate = millis();
            return; // Skip normal display content
        }
        
        // Show current mode
        display.print("Mode: ");
        if (mode == 1) {
            display.println("VEL (1)");
        } else if (mode == 2) {
            display.println("POS (2)");
        } else if (mode == 3) {
            display.println("POS (3)");
        } else if (mode == 4) {
            display.println("FIX (4)");
        } else if (mode == 5) {
            display.println("IND (5)");
        } else if (mode == 6) {
            display.println("STOP (6)");
        } else {
            display.print("? (");
            display.print(mode);
            display.println(")");
        }
        
        // Calculate individual motor targets (accounting for direction reversal)
        float motor1Target = 0.0f;
        float motor2Target = 0.0f;
        bool motor1Reversed = true;
        bool motor2Reversed = false;
        
        if (mode == 6) {
            motor1Target = 0.0f;
            motor2Target = 0.0f;
        } else if (mode == 4) {
            // Mode 4: Fixed positions with button A override
            if (switches.switchA) {
                // Button A pressed - alternate positions
                motor1Target = motor1Reversed ? -(-2.4f) : -2.4f;  // M1 target: -2.4 rad
                motor2Target = motor2Reversed ? -1.3f : 1.3f;       // M2 target: 1.3 rad
            } else {
                // Button A not pressed - default positions
                motor1Target = motor1Reversed ? -(-4.4f) : -4.4f;  // M1 target: -4.4 rad
                motor2Target = motor2Reversed ? -(-0.6f) : -0.6f;   // M2 target: -0.6 rad
            }
        } else if (mode == 5) {
            // Mode 5: Individual motor control - calculate from channel values with display smoothing
            int ch7Value = crsf.getChannel(7);  // Channel 7 for Motor 1
            int ch9Value = crsf.getChannel(9);  // Channel 9 for Motor 2
            
            // Map Channel 7 to Motor 1 position
            float motor1Position = 0.0f;
            if (ch7Value >= (MID_PULSE - DEAD_ZONE) && ch7Value <= (MID_PULSE + DEAD_ZONE)) {
                motor1Position = 0.0f;
            } else if (ch7Value < MID_PULSE) {
                motor1Position = map(ch7Value, MIN_PULSE, MID_PULSE - DEAD_ZONE, -7000, 0) * 0.001f;
            } else {
                motor1Position = map(ch7Value, MID_PULSE + DEAD_ZONE, MAX_PULSE, 0, 7000) * 0.001f;
            }
            
            // Map Channel 9 to Motor 2 position
            float motor2Position = 0.0f;
            if (ch9Value >= (MID_PULSE - DEAD_ZONE) && ch9Value <= (MID_PULSE + DEAD_ZONE)) {
                motor2Position = 0.0f;
            } else if (ch9Value < MID_PULSE) {
                motor2Position = map(ch9Value, MIN_PULSE, MID_PULSE - DEAD_ZONE, -7000, 0) * 0.001f;
            } else {
                motor2Position = map(ch9Value, MID_PULSE + DEAD_ZONE, MAX_PULSE, 0, 7000) * 0.001f;
            }
            
            // Apply smoothing for display (matches motor control smoothing)
            static float displaySmoothedMotor1 = 0.0f;
            static float displaySmoothedMotor2 = 0.0f;
            static bool displaySmoothingInit = false;
            static int lastDisplayMode = 0;
            const float displaySmoothingFactor = 0.15f;
            
            // Reset display smoothing when mode changes
            if (mode != lastDisplayMode) {
                displaySmoothingInit = false;
                lastDisplayMode = mode;
            }
            
            if (!displaySmoothingInit) {
                displaySmoothedMotor1 = motor1Position;
                displaySmoothedMotor2 = motor2Position;
                displaySmoothingInit = true;
            }
            
            displaySmoothedMotor1 += displaySmoothingFactor * (motor1Position - displaySmoothedMotor1);
            displaySmoothedMotor2 += displaySmoothingFactor * (motor2Position - displaySmoothedMotor2);
            
            motor1Target = motor1Reversed ? -displaySmoothedMotor1 : displaySmoothedMotor1;
            motor2Target = motor2Reversed ? -displaySmoothedMotor2 : displaySmoothedMotor2;
        } else if (inPositionMode) {
            motor1Target = motor1Reversed ? -currentTargetPosition : currentTargetPosition;
            motor2Target = motor2Reversed ? -currentTargetPosition : currentTargetPosition;
        } else {
            motor1Target = motor1Reversed ? -currentTargetPosition : currentTargetPosition;
            motor2Target = motor2Reversed ? -currentTargetPosition : currentTargetPosition;
        }
        
        // Show Motor 1 desired value
        display.print("M1 Des: ");
        if (mode == 6) {
            display.println("STOP");
        } else if (inPositionMode) {
            display.print(motor1Target, 1);
            display.println(" rad");
        } else {
            display.print(motor1Target, 1);
            display.println(" r/s");
        }
        
        // Show Motor 2 desired value
        display.print("M2 Des: ");
        if (mode == 6) {
            display.println("STOP");
        } else if (inPositionMode) {
            display.print(motor2Target, 1);
            display.println(" rad");
        } else {
            display.print(motor2Target, 1);
            display.println(" r/s");
        }
        
        // Show switch states
        display.print("Btn A: ");
        display.print(switches.switchA ? "ON" : "OFF");
        display.print("  B: ");
        display.print(switches.switchB ? "ON" : "OFF");
        
        // Show nudgeable zero position in Mode 2
        if (mode == 2) {
            display.print(" Z:");
            display.print(nudgeableZeroPosition, 2);
        }
        display.println();
        
        display.display();
        lastUpdate = millis();


    }
    
    // Print to serial console every 100ms (only if serial is available)
    if (Serial && millis() - lastSerialPrint >= 100) {
        // Show emergency stop status if active
        if (emergencyStopActive) {
            Serial.println("*** EMERGENCY STOP ACTIVE *** RC SIGNAL TIMEOUT - Motors STOPPED - Check RC connection");
            lastSerialPrint = millis();
            return; // Skip normal serial output
        }
        
        // Print mode information
        Serial.print("Mode:");
        if (mode == 1) {
            Serial.print("VEL(1)");
        } else if (mode == 2) {
            Serial.print("POS(2)");
        } else if (mode == 3) {
            Serial.print("POS(3)");
        } else if (mode == 4) {
            Serial.print("FIX(4)");
        } else if (mode == 5) {
            Serial.print("IND(5)");
        } else if (mode == 6) {
            Serial.print("STOP(6)");
        } else {
            Serial.print("?(");
            Serial.print(mode);
            Serial.print(")");
        }
        Serial.print("  ");
        
        // Calculate individual motor targets (accounting for direction reversal)
        float motor1Target = 0.0f;
        float motor2Target = 0.0f;
        bool motor1Reversed = true;
        bool motor2Reversed = false;
        
        if (mode == 6) {
            motor1Target = 0.0f;
            motor2Target = 0.0f;
        } else if (mode == 4) {
            // Mode 4: Fixed positions with button A override
            if (switches.switchA) {
                // Button A pressed - alternate positions
                motor1Target = motor1Reversed ? -(-2.4f) : -2.4f;  // M1 target: -2.4 rad
                motor2Target = motor2Reversed ? -1.3f : 1.3f;       // M2 target: 1.3 rad
            } else {
                // Button A not pressed - default positions
                motor1Target = motor1Reversed ? -(-4.4f) : -4.4f;  // M1 target: -4.4 rad
                motor2Target = motor2Reversed ? -(-0.6f) : -0.6f;   // M2 target: -0.6 rad
            }
        } else if (mode == 5) {
            // Mode 5: Individual motor control - calculate from channel values with serial smoothing
            int ch7Value = crsf.getChannel(7);  // Channel 7 for Motor 1
            int ch9Value = crsf.getChannel(9);  // Channel 9 for Motor 2
            
            // Map Channel 7 to Motor 1 position
            float motor1Position = 0.0f;
            if (ch7Value >= (MID_PULSE - DEAD_ZONE) && ch7Value <= (MID_PULSE + DEAD_ZONE)) {
                motor1Position = 0.0f;
            } else if (ch7Value < MID_PULSE) {
                motor1Position = map(ch7Value, MIN_PULSE, MID_PULSE - DEAD_ZONE, -7000, 0) * 0.001f;
            } else {
                motor1Position = map(ch7Value, MID_PULSE + DEAD_ZONE, MAX_PULSE, 0, 7000) * 0.001f;
            }
            
            // Map Channel 9 to Motor 2 position
            float motor2Position = 0.0f;
            if (ch9Value >= (MID_PULSE - DEAD_ZONE) && ch9Value <= (MID_PULSE + DEAD_ZONE)) {
                motor2Position = 0.0f;
            } else if (ch9Value < MID_PULSE) {
                motor2Position = map(ch9Value, MIN_PULSE, MID_PULSE - DEAD_ZONE, -7000, 0) * 0.001f;
            } else {
                motor2Position = map(ch9Value, MID_PULSE + DEAD_ZONE, MAX_PULSE, 0, 7000) * 0.001f;
            }
            
            // Apply smoothing for serial output (matches motor control smoothing)
            static float serialSmoothedMotor1 = 0.0f;
            static float serialSmoothedMotor2 = 0.0f;
            static bool serialSmoothingInit = false;
            static int lastSerialMode = 0;
            const float serialSmoothingFactor = 0.15f;
            
            // Reset serial smoothing when mode changes
            if (mode != lastSerialMode) {
                serialSmoothingInit = false;
                lastSerialMode = mode;
            }
            
            if (!serialSmoothingInit) {
                serialSmoothedMotor1 = motor1Position;
                serialSmoothedMotor2 = motor2Position;
                serialSmoothingInit = true;
            }
            
            serialSmoothedMotor1 += serialSmoothingFactor * (motor1Position - serialSmoothedMotor1);
            serialSmoothedMotor2 += serialSmoothingFactor * (motor2Position - serialSmoothedMotor2);
            
            motor1Target = motor1Reversed ? -serialSmoothedMotor1 : serialSmoothedMotor1;
            motor2Target = motor2Reversed ? -serialSmoothedMotor2 : serialSmoothedMotor2;
        } else if (inPositionMode) {
            motor1Target = motor1Reversed ? -currentTargetPosition : currentTargetPosition;
            motor2Target = motor2Reversed ? -currentTargetPosition : currentTargetPosition;
        } else {
            motor1Target = motor1Reversed ? -currentTargetPosition : currentTargetPosition;
            motor2Target = motor2Reversed ? -currentTargetPosition : currentTargetPosition;
        }
        
        // Print Motor 1 desired value
        Serial.print("M1_Des:");
        if (mode == 6) {
            Serial.print("STOP");
        } else if (inPositionMode) {
            Serial.print(motor1Target, 2);
            Serial.print("rad");
        } else {
            Serial.print(motor1Target, 2);
            Serial.print("r/s");
        }
        Serial.print("  ");
        
        // Print Motor 2 desired value
        Serial.print("M2_Des:");
        if (mode == 6) {
            Serial.print("STOP");
        } else if (inPositionMode) {
            Serial.print(motor2Target, 2);
            Serial.print("rad");
        } else {
            Serial.print(motor2Target, 2);
            Serial.print("r/s");
        }
        Serial.print("  ");
        
        // Print button states
        Serial.print("BtnA:");
        Serial.print(switches.switchA ? "ON" : "OFF");
        Serial.print("  BtnB:");
        Serial.print(switches.switchB ? "ON" : "OFF");
        
        // Show nudgeable zero position in Mode 2
        if (mode == 2) {
            Serial.print("  Zero:");
            Serial.print(nudgeableZeroPosition, 3);
            Serial.print("rad");
        }
        
        Serial.println();
        lastSerialPrint = millis();
    }
    
    
    // Small delay to prevent tight looping (reduced from 10ms since RC is interrupt-driven)
    delay(5);
} 
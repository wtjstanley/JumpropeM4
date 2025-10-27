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

// ----- System Status Variables -----
static bool canAvailable = false;
struct SwitchStates {
        bool switchA;
        bool switchB;
};

enum State current_state = TO_MODE_CONTROL;


// ----- Setup -----
void setup() {
    // Initialize serial communication with timeout
    Serial.begin(115200);
    
    Serial1.begin(CRSF_BAUDRATE);
    if (!Serial1) while (1) Serial.println("Invalid crsfSerial configuration");
    crsf.begin(Serial1);
    
    // Initialize NeoPixel
    pixels.begin();
    pixels.setBrightness(50);
    pixels.clear();
    pixels.setPixelColor(0, pixels.Color(50, 50, 0)); // Yellow while initializing
    pixels.show();
   
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
        pixels.setPixelColor(0, pixels.Color(50, 0, 50));
        pixels.show();
    }

    delay(5000);
    Serial.println("Arming");

    motor1.resetFaults();
    motor2.resetFaults();
    Serial.println("reset faults");
    delay(200);
    motor1.setZeroFlag(1);
    motor2.setZeroFlag(1);

    bool zero1Success = motor1.setMechanicalZero();
    bool zero2Success = motor2.setMechanicalZero();

    motor1.setModeVelocity();
    motor2.setModeVelocity();
    Serial.println("setMoveVelocity");
    delay(200);

    motor1.enable();
    motor2.enable();
    Serial.println("enabled");
    delay(200);

    // motor1.setZeroFlag(1);
    // Serial.println("setZeroFlag");
    // delay(100);

    
    // motor2.resetFaults();
    // delay(100);
    
    // motor1.enable();
    // motor2.enable();
    // delay(200);  // Give motors time to enable
    
    
    
    // if (!motor1.setZeroFlag(1)) {
    //     Logger::warning("Failed to set zero flag for motor1");
    // }
    // if (!motor2.setZeroFlag(1)) {
    //     Logger::warning("Failed to set zero flag for motor2");
    // }
    // delay(200);
    
    // Logger::info("Setting mechanical zero positions");
    // bool zero1Success = motor1.setMechanicalZero();
    // bool zero2Success = motor2.setMechanicalZero();
    
   
    // Logger::info("Step 3: Setting position mode");
    // if (!motor1.setModePositionPP(25.0f, 200.0f, 40.0f)) {
    //     Logger::error("Failed to set motor1 to position mode!");
    // }
    // if (!motor2.setModePositionPP(25.0f, 200.0f, 40.0f)) {
    //     Logger::error("Failed to set motor2 to position mode!");
    // }
    // delay(500);  // Allow time for mode setting to complete
    
    // // Step 4: Enable active reporting for position feedback
    // Logger::info("Step 4: Enabling active reporting");
    
    // // Enable active reporting for motor1 with fallback
    // bool reportingEnabled1 = motor1.setActiveReporting(true);
    
    // // Enable active reporting for motor2 with fallback
    // bool reportingEnabled2 = motor2.setActiveReporting(true);
    
    // delay(200);
    
   
    // motor1.setPosition(0.0f);
    // motor2.setPosition(0.0f);
   
    
}

State get_requested_mode(){
    int mode_reading = crsf.getChannel(CH_MODE);
    Serial.print(mode_reading);
    Serial.print("  ");
    if(mode_reading < 1250){
        return VELOCITY_MODE;
    }else if(mode_reading >= 1250 && mode_reading < 1500){
        return POSITION_SETPOINT_MODE;
    }else if(mode_reading >= 1500 && mode_reading < 1750){
        return POSITION_STOW_MODE;
    }else{
        return POSITION_ANALOG_MODE;
    }
}
    

void stateMachine(){
    switch(current_state){
        case TO_ESTOP:{
                Logger::info("ESTOPPING");
                pixels.setPixelColor(0, pixels.Color(50, 0, 0)); // Red Sad
                pixels.show();
                motor1.disable();
                motor2.disable();
                current_state = ESTOP;
        }
            break;

        case ESTOP:{
                if(crsf.isLinkUp() && crsf.getChannel(CH_ESTOP) > 1500 ){
                    current_state = TO_MODE_CONTROL;
                    Logger::info("Leaving ESTOP");
                }
            }
            break;

        case TO_MODE_CONTROL:{
                enum State next_mode = get_requested_mode();
                Logger::info("Next State");
                pixels.setPixelColor(0, pixels.Color(0, 50, 0)); // Green for normal operation
                pixels.show();
                motor1.resetFaults();
                motor2.resetFaults();
                if(next_mode == VELOCITY_MODE){
                    Logger::info("Velocity");
                    motor1.setModeVelocity();
                    motor2.setModeVelocity();
                }else{
                    Logger::info("Position");
                    motor1.setModePositionPP(POSITION_SPEED_LIMIT, POSITION_ACCELERATION, MOTOR_CURRENT_LIMIT);
                    motor2.setModePositionPP(POSITION_SPEED_LIMIT, POSITION_ACCELERATION, MOTOR_CURRENT_LIMIT);
                }
                motor1.enable();
                motor2.enable();
                current_state = next_mode;
            }
            break;

        case VELOCITY_MODE:{
                // Boilerplate if statement to change states when necessary
                if(get_requested_mode() != current_state){
                    current_state = TO_MODE_CONTROL;
                }

                float velL = (crsf.getChannel(CH_L_ARM)-1500)/25.0;
                float velR = (crsf.getChannel(CH_R_ARM)-1500)/25.0;
                //Apply a small deadband 
                if(abs(velL) < 0.1){ velL = 0.0; }
                if(abs(velR) < 0.1){ velR = 0.0; }

                motor1.setVelocity(velL);
                motor2.setVelocity(velR);
            }
            break;
        
        case POSITION_SETPOINT_MODE:{
                // Boilerplate if statement to change states when necessary
                if(get_requested_mode() != current_state){
                    current_state = TO_MODE_CONTROL;
                }
            }
            break;
        case POSITION_STOW_MODE:{
                // Boilerplate if statement to change states when necessary
                if(get_requested_mode() != current_state){
                    current_state = TO_MODE_CONTROL;
                }
            }
            break;
        case POSITION_ANALOG_MODE:{
                // Boilerplate if statement to change states when necessary
                if(get_requested_mode() != current_state){
                    current_state = TO_MODE_CONTROL;
                }
                
                float posL = M_PI*(crsf.getChannel(CH_L_ARM)-1500)/500.0;
                float posR = M_PI*(crsf.getChannel(CH_R_ARM)-1500)/500.0;

                motor1.setPosition(posL);
                motor2.setPosition(posR);
            }
            break;
    }
}

// ----- Main Loop -----
void loop() {
    //Any checks that should happen regardless of state
    crsf.update();

    
    //Set mode to estop last before running state machine as that takes highest priority
    if((!crsf.isLinkUp() || crsf.getChannel(CH_ESTOP) < 1500) && ESTOP != current_state){
        current_state = TO_ESTOP;
    }

    stateMachine();
    Serial.println(current_state);
    delay(50);
} 
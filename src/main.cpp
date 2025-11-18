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

enum State current_state = TO_MODE_CONTROL;

RS03Motor::Feedback motor1_feedback;
RS03Motor::Feedback motor2_feedback;

void torque_mit_motors(float torque_left, float torque_right){
    // setMitCommand(float position_rad, float velocity_rad_s, float kp, float kd, float torque_nm);
    motor1.setMitCommand(0.0, 0.0, 0.0, 0.0, torque_left);
    motor2.setMitCommand(0.0, 0.0, 0.0, 0.0, torque_right);
}

void velocity_mit_motors(float vel_left, float vel_right){
    // setMitCommand(float position_rad, float velocity_rad_s, float kp, float kd, float torque_nm);
    motor1.setMitCommand(0.0, vel_left, 0.0, 0.5, 0.0);
    motor2.setMitCommand(0.0, vel_right, 0.0, 0.5, 0.0);
}

void position_mit_motors(float position_left, float position_right){
    // setMitCommand(float position_rad, float velocity_rad_s, float kp, float kd, float torque_nm);
    motor1.setMitCommand(position_left, 0.0, 1.2, 0.06, 0.0);
    motor2.setMitCommand(position_right, 0.0, 1.2, 0.06, 0.0);
}

// ----- Setup -----
void setup() {
    // Initialize serial communication with timeout
    Serial.begin(115200);
    
    // Set up CRSF communication with ELRS RX
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

    // Turns out this delay is super important otherwise the mcu might send signals before the motors are ready for it.
    // Might be able to slim down the time in the future
    delay(5000);
    Serial.println("Arming");

    motor1.resetFaults();
    motor2.resetFaults();
    Serial.println("reset faults");
    delay(200);
    motor1.setZeroFlag(1);
    motor2.setZeroFlag(1);

    motor1.setModeMit();
    motor2.setModeMit();
    Serial.println("setModeMIT");
    delay(200);

    motor1.enable();
    motor2.enable();
    velocity_mit_motors(0.0, 0.0);
    Serial.println("enabled");
    delay(200);

}

// This function reads CH_MODE and depending on the value will return what the user requested state is
State get_requested_mode(){
    int mode_reading = crsf.getChannel(CH_MODE);
    if(mode_reading < 1250){
        return VELOCITY_MODE;
    }else if(mode_reading >= 1250 && mode_reading < 1500){
        return TORQUE_MODE;
    }else if(mode_reading >= 1500 && mode_reading < 1750){
        return POSITION_ANALOG_MODE;
    }else{
        return SET_MECHANICAL_ZERO_MODE;
    }
}
    
// This runs all of the logic for transitioning through different control states
void stateMachine(){
    switch(current_state){
        // This state runs any single time commands when going into the ESTOP state to prevent potentially hammering the CAN bus
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
                // Check to make sure that the RX is both connected, and is armed via CH_ESTOP
                if(crsf.isLinkUp() && crsf.getChannel(CH_ESTOP) > 1500 ){
                    current_state = TO_MODE_CONTROL;
                    Logger::info("Leaving ESTOP");
                }
            }
            break;

        // This state will run any single time commands before transitioning into the next control state (or estop)
        case TO_MODE_CONTROL:{
                
                enum State next_mode = get_requested_mode();
                Logger::info("Next State");
                current_state = next_mode;
                pixels.setPixelColor(0, pixels.Color(0, 50, 0)); // Green for normal operation
                pixels.show();

                motor1.resetFaults();
                motor2.resetFaults();
                motor1.enable();
                motor2.enable();
            }
            break;

        // Torque or current controller mode
        case TORQUE_MODE:{
                // Boilerplate if statement to change states when necessary
                if(get_requested_mode() != current_state){
                    current_state = TO_MODE_CONTROL;
                }
                pixels.setPixelColor(0, pixels.Color(50, 0, 50)); // Magenta for Torque
                pixels.show();

                float velL = (crsf.getChannel(CH_L_ARM)-1500)/100.0;
                float velR = (crsf.getChannel(CH_R_ARM)-1500)/100.0;
                torque_mit_motors(velL, velR);
            }
            break;

        // Velocity mode just maps sticks to velocities
        case VELOCITY_MODE:{
                // Boilerplate if statement to change states when necessary
                if(get_requested_mode() != current_state){
                    current_state = TO_MODE_CONTROL;
                }
                pixels.setPixelColor(0, pixels.Color(0, 50, 0)); // Green for velocity
                pixels.show();

                float velL = (crsf.getChannel(CH_L_ARM)-1500)/25.0;
                float velR = (crsf.getChannel(CH_R_ARM)-1500)/25.0;
                velocity_mit_motors(velL, velR);
            }
            break;
        
        // Currently unused state, was present in jumprope V1, and easy to add back
        case POSITION_SETPOINT_MODE:{
                // Boilerplate if statement to change states when necessary
                if(get_requested_mode() != current_state){
                    current_state = TO_MODE_CONTROL;
                }
            }
            break;

        // Currently unused state, was present in jumprope V1, and easy to add back
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
                pixels.setPixelColor(0, pixels.Color(0, 50, 50)); // Teal for position
                pixels.show();

                float posL = M_PI*(crsf.getChannel(CH_L_ARM)-1500)/500.0;
                float posR = M_PI*(crsf.getChannel(CH_R_ARM)-1500)/500.0;
                position_mit_motors(posL, posR);
            }
            break;

        // Position control mode
        // TODO: add logic to prevent excessive unwrapping through well timed mechanical zeros and maintaining a mechanical zero trim value
        case SET_MECHANICAL_ZERO_MODE:{
                // Boilerplate if statement to change states when necessary
                if(get_requested_mode() != current_state){
                    current_state = TO_MODE_CONTROL;
                }
                pixels.setPixelColor(0, pixels.Color(0, 0, 50)); // Blue for zeroing
                pixels.show();
                bool zero1Success = motor1.setMechanicalZero();
                bool zero2Success = motor2.setMechanicalZero();
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

    motor1_feedback = motor1.getLastFeedback();
    motor2_feedback = motor2.getLastFeedback();

    
    stateMachine();
    //Serial.println(current_state);
} 
#ifndef SPEKTRUM_SATELLITE_READER_H
#define SPEKTRUM_SATELLITE_READER_H

#include <Arduino.h>
#include "Arduino-CRSF.h"


class SpektrumSatelliteReader {
public:
    // Constructor with serial port and optional parameters
    SpektrumSatelliteReader(HardwareSerial& serial, int numChannels = 8, int minValue = 1000, int maxValue = 2000, int defaultValue = 1500);
    
    // Destructor
    ~SpektrumSatelliteReader();
    
    // Initialize the Spektrum satellite reader
    bool begin();
    
    // Initialize the Spektrum satellite reader in bind mode (uses same pin as serial RX)
    bool beginBindMode();
    
    // Initialize timer-based interrupt processing (SAMD51 specific)
    bool beginInterruptMode(int timerFrequencyHz = 500);  // Default 500Hz = 2ms intervals
    
    // Stop interrupt mode
    void endInterruptMode();
    
    // Get channel value (in microseconds, mapped to 1000-2000 range like CPPM)
    int getChannel(int channel) const;
    
    // Returns true if signal has been detected since initialization
    bool isReceiving() const;
    
    // Returns time since the last frame was received (in milliseconds)
    unsigned long lastFrameTime() const;
    
    // Returns number of channels configured
    int getChannelCount() const { return _numChannels; }
    
    // Map channel value to a custom range (same as CPPM interface)
    static float mapToRange(int channelValue, float minOutput, float maxOutput,
                           int minPulse = 1000, int maxPulse = 2000);
    
    // Call this to update and process incoming data
    // Returns true if signal is valid
    bool update();
    
    // Interrupt-safe update method (called from timer interrupt)
    void updateFromInterrupt();
    
    // Decode MODE from channel 4 (0-based index 3)
    int getModeFromChannel4() const;
    
    // Decode switch states from channel 6 (0-based index 5)
    struct SwitchStates {
        bool switchA;
        bool switchB;
    };
    SwitchStates getSwitchStatesFromChannel6() const;
    
    // Decode Channel 8 state (0-based index 7)
    enum Channel8State {
        CH8_LOW,      // Low position
        CH8_MIDDLE,   // Middle position
        CH8_HIGH,     // High position
        CH8_UNKNOWN   // Unknown/invalid state
    };
    Channel8State getChannel8State() const;
    
    // Get frame statistics for debugging
    struct FrameStats {
        unsigned long totalFrames;
        unsigned long validFrames;
        unsigned long formatDetectSamples;
        unsigned long lastFrameTime;
        bool is11Bit;
        int detectedChannels;
    };
    FrameStats getFrameStats() const { return _frameStats; }
    CRSF crsf;

private:
    // Spektrum satellite protocol constants
    static const int DSM_FRAME_SIZE = 16;  // 16 bytes per frame
    static const int DSM_FRAME_CHANNELS = 7;  // 7 channels per frame max
    static const unsigned long CRSF_BAUD_RATE = 420000;
    static const unsigned long FRAME_GAP_MS = 5;  // 5ms gap indicates new frame
    static const unsigned long SIGNAL_TIMEOUT_MS = 200; // Signal timeout
    
    // Decode states (based on ArduPilot)
    enum dsm_decode_state {
        DSM_DECODE_STATE_DESYNC = 0,
        DSM_DECODE_STATE_SYNC
    };
    
    HardwareSerial& _serial;
    int _numChannels;
    int _minValue;
    int _maxValue;
    int _defaultValue;
    
    int* _channelValues;        // Current channel values
    bool _receivingSignal;
    unsigned long _lastFrameTime;
    unsigned long _lastRxTime;
    
    // Frame processing state
    uint8_t _frameBuffer[DSM_FRAME_SIZE];
    int _frameIndex;
    dsm_decode_state _decodeState;
    
    // Format detection (based on ArduPilot)
    int _channelShift;  // 10 or 11 bit mode
    uint32_t _cs10, _cs11;  // Channel masks for format detection
    int _formatSamples;
    
    // Frame statistics for debugging
    FrameStats _frameStats;
    
    // Interrupt mode state
    bool _interruptMode;
    static SpektrumSatelliteReader* _instance;  // For interrupt callback
    
    // Process a complete frame (ArduPilot approach)
    bool processFrame(uint32_t frameTimeMs);
    
    // Process single byte (ArduPilot approach)
    bool processByte(uint32_t frameTimeMs, uint8_t b);
    
    // Decode channel from raw data (ArduPilot approach)
    bool decodeChannel(uint16_t raw, unsigned shift, unsigned *channel, unsigned *value);
    
    // Format detection (ArduPilot approach)
    void detectFormat(bool reset);
    
    // Decode frame data (ArduPilot approach)
    bool decodeFrame(uint32_t frameTimeMs, const uint8_t* frame);
    
    // Static interrupt handler
    static void timerInterruptHandler();
};

#endif // SPEKTRUM_SATELLITE_READER_H 
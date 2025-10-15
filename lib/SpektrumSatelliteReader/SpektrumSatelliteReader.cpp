#include "SpektrumSatelliteReader.h"

// Static instance for interrupt callback
SpektrumSatelliteReader* SpektrumSatelliteReader::_instance = nullptr;

SpektrumSatelliteReader::SpektrumSatelliteReader(HardwareSerial& serial, int numChannels, int minValue, int maxValue, int defaultValue)
    : _serial(serial), _numChannels(numChannels), _minValue(minValue), _maxValue(maxValue), _defaultValue(defaultValue) {
    
    _channelValues = new int[_numChannels];
    
    // Initialize all channels to default value
    for (int i = 0; i < _numChannels; i++) {
        _channelValues[i] = _defaultValue;
    }
    
    _receivingSignal = false;
    _lastFrameTime = 0;
    _lastRxTime = 0;
    _frameIndex = 0;
    _decodeState = DSM_DECODE_STATE_DESYNC;
    _channelShift = 0;  // Unknown format initially
    _cs10 = 0;
    _cs11 = 0;
    _formatSamples = 0;
    _interruptMode = false;
    
    // Initialize frame statistics
    _frameStats.totalFrames = 0;
    _frameStats.validFrames = 0;
    _frameStats.formatDetectSamples = 0;
    _frameStats.lastFrameTime = 0;
    _frameStats.is11Bit = false;
    _frameStats.detectedChannels = 0;
}

SpektrumSatelliteReader::~SpektrumSatelliteReader() {
    if (_interruptMode) {
        endInterruptMode();
    }
    delete[] _channelValues;
}

bool SpektrumSatelliteReader::begin() {
    crsf.begin(&Serial1, CRSF_BAUD_RATE);
    
    _frameIndex = 0;
    _decodeState = DSM_DECODE_STATE_DESYNC;
    _receivingSignal = false;
    _lastFrameTime = millis();
    _lastRxTime = millis();
    
    return true;
}

bool SpektrumSatelliteReader::beginInterruptMode(int timerFrequencyHz) {
    if (_interruptMode) {
        return true; // Already in interrupt mode
    }
    
    // Set the static instance for interrupt callback
    _instance = this;
    
    #ifdef ARDUINO_SAMD_VARIANT
    // SAMD51 Timer setup for interrupt processing
    // Use TC3 (Timer/Counter 3) for RC input processing
    
    // Enable GCLK for TC3
    GCLK->PCHCTRL[TC3_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK0_Val | (1 << GCLK_PCHCTRL_CHEN_Pos);
    while (GCLK->SYNCBUSY.reg > 0);
    
    // Enable TC3 in PM
    MCLK->APBCMASK.reg |= MCLK_APBCMASK_TC3;
    
    // Reset TC3
    TC3->COUNT16.CTRLA.bit.SWRST = 1;
    while (TC3->COUNT16.SYNCBUSY.bit.SWRST);
    
    // Configure TC3 for 16-bit mode with prescaler
    TC3->COUNT16.CTRLA.reg = TC_CTRLA_MODE_COUNT16 | TC_CTRLA_PRESCALER_DIV64;
    
    // Calculate compare value for desired frequency
    // GCLK0 is typically 120MHz, with DIV64 prescaler = 1.875MHz
    // For 500Hz: 1875000 / 500 = 3750
    uint16_t compareValue = (1875000 / timerFrequencyHz) - 1;
    TC3->COUNT16.CC[0].reg = compareValue;
    while (TC3->COUNT16.SYNCBUSY.bit.CC0);
    
    // Enable compare match interrupt
    TC3->COUNT16.INTENSET.bit.MC0 = 1;
    
    // Enable TC3 interrupt in NVIC
    NVIC_EnableIRQ(TC3_IRQn);
    NVIC_SetPriority(TC3_IRQn, 1); // High priority for RC processing
    
    // Enable TC3
    TC3->COUNT16.CTRLA.bit.ENABLE = 1;
    while (TC3->COUNT16.SYNCBUSY.bit.ENABLE);
    
    _interruptMode = true;
    return true;
    
    #else
    // Non-SAMD platform - fall back to polling mode
    return false;
    #endif
}

void SpektrumSatelliteReader::endInterruptMode() {
    if (!_interruptMode) {
        return;
    }
    
    #ifdef ARDUINO_SAMD_VARIANT
    // Disable TC3 interrupt
    NVIC_DisableIRQ(TC3_IRQn);
    
    // Disable TC3
    TC3->COUNT16.CTRLA.bit.ENABLE = 0;
    while (TC3->COUNT16.SYNCBUSY.bit.ENABLE);
    
    // Reset TC3
    TC3->COUNT16.CTRLA.bit.SWRST = 1;
    while (TC3->COUNT16.SYNCBUSY.bit.SWRST);
    #endif
    
    _interruptMode = false;
    _instance = nullptr;
}

int SpektrumSatelliteReader::getChannel(int channel) const {
    if (channel < 0 || channel >= _numChannels) {
        return _defaultValue;
    }
    return _channelValues[channel];
}

bool SpektrumSatelliteReader::isReceiving() const {
    return _receivingSignal && (millis() - _lastFrameTime < SIGNAL_TIMEOUT_MS);
}

unsigned long SpektrumSatelliteReader::lastFrameTime() const {
    return _lastFrameTime;
}

float SpektrumSatelliteReader::mapToRange(int channelValue, float minOutput, float maxOutput, int minPulse, int maxPulse) {
    channelValue = constrain(channelValue, minPulse, maxPulse);
    return minOutput + (maxOutput - minOutput) * (channelValue - minPulse) / (maxPulse - minPulse);
}

bool SpektrumSatelliteReader::update() {
    if (_interruptMode) {
        // In interrupt mode, just check signal status
        uint32_t now = millis();
        if (now - _lastFrameTime > SIGNAL_TIMEOUT_MS) {
            _receivingSignal = false;
        }
        return isReceiving();
    } else {
        // Original polling implementation
        uint32_t now = millis();
        
        // Process all available bytes
        while (_serial.available()) {
            uint8_t b = _serial.read();
            if (processByte(now, b)) {
                _receivingSignal = true;
                _lastFrameTime = now;
            }
        }
        
        // Check for signal timeout
        if (now - _lastFrameTime > SIGNAL_TIMEOUT_MS) {
            _receivingSignal = false;
        }
        
        return isReceiving();
    }
}

void SpektrumSatelliteReader::updateFromInterrupt() {
    // This method is called from timer interrupt
    // Keep it minimal and fast
    uint32_t now = millis();
    
    // Process available bytes (limit to prevent long interrupt time)
    int bytesProcessed = 0;
    const int maxBytesPerInterrupt = 8; // Limit processing time
    
    while (_serial.available() && bytesProcessed < maxBytesPerInterrupt) {
        uint8_t b = _serial.read();
        if (processByte(now, b)) {
            _receivingSignal = true;
            _lastFrameTime = now;
        }
        bytesProcessed++;
    }
    
    // Check for signal timeout
    if (now - _lastFrameTime > SIGNAL_TIMEOUT_MS) {
        _receivingSignal = false;
    }
}

// Static interrupt handler for SAMD51 TC3
void SpektrumSatelliteReader::timerInterruptHandler() {
    if (_instance != nullptr) {
        _instance->updateFromInterrupt();
    }
    
    #ifdef ARDUINO_SAMD_VARIANT
    // Clear interrupt flag
    TC3->COUNT16.INTFLAG.bit.MC0 = 1;
    #endif
}
//////////////////////////////////
bool SpektrumSatelliteReader::processByte(uint32_t frameTimeMs, uint8_t b) {
    bool frameComplete = false;
    
    // Check for buffer overflow
    if (_frameIndex >= DSM_FRAME_SIZE) {
        _frameIndex = 0;
        _decodeState = DSM_DECODE_STATE_DESYNC;
    }
    
    switch (_decodeState) {
    case DSM_DECODE_STATE_DESYNC:
        // Look for frame start (5ms gap indicates new frame)
        if ((frameTimeMs - _lastRxTime) >= FRAME_GAP_MS) {
            _decodeState = DSM_DECODE_STATE_SYNC;
            _frameIndex = 0;
            _frameBuffer[_frameIndex++] = b;
        }
        break;
        
    case DSM_DECODE_STATE_SYNC:
        // Check if this is a new frame (5ms gap)
        if ((frameTimeMs - _lastRxTime) >= FRAME_GAP_MS && _frameIndex > 0) {
            // Process the previous frame if we have one
            if (_frameIndex == DSM_FRAME_SIZE) {
                frameComplete = processFrame(frameTimeMs);
            }
            // Start new frame
            _frameIndex = 0;
        }
        
        _frameBuffer[_frameIndex++] = b;
        
        // Check if we have a complete frame
        if (_frameIndex == DSM_FRAME_SIZE) {
            frameComplete = processFrame(frameTimeMs);
            _frameIndex = 0;
        }
        break;
    }
    
    _lastRxTime = frameTimeMs;
    return frameComplete;
}

bool SpektrumSatelliteReader::processFrame(uint32_t frameTimeMs) {
    _frameStats.totalFrames++;
    
    // Reset format detection if we haven't seen frames for a while
    if ((frameTimeMs - _frameStats.lastFrameTime) > 200 && _channelShift != 0) {
        detectFormat(true);
    }
    
    _frameStats.lastFrameTime = frameTimeMs;
    
    // If format unknown, try to detect it
    if (_channelShift == 0) {
        detectFormat(false);
        return false;  // Don't decode until format is known
    }
    
    return decodeFrame(frameTimeMs, _frameBuffer);
}

bool SpektrumSatelliteReader::decodeChannel(uint16_t raw, unsigned shift, unsigned *channel, unsigned *value) {
    if (raw == 0xffff) {
        return false;
    }
    
    *channel = (raw >> shift) & 0xf;
    uint16_t data_mask = (1 << shift) - 1;
    *value = raw & data_mask;
    
    return true;
}

void SpektrumSatelliteReader::detectFormat(bool reset) {
    if (reset) {
        _cs10 = 0;
        _cs11 = 0;
        _formatSamples = 0;
        _channelShift = 0;
        _frameStats.formatDetectSamples = 0;
        return;
    }
    
    // Scan channels in current frame for both 10-bit and 11-bit modes
    for (unsigned i = 0; i < DSM_FRAME_CHANNELS; i++) {
        const uint8_t *dp = &_frameBuffer[2 + (2 * i)];
        uint16_t raw = (dp[0] << 8) | dp[1];
        unsigned channel, value;
        
        // Try 10-bit decode
        if (decodeChannel(raw, 10, &channel, &value) && (channel < 16)) {
            _cs10 |= (1 << channel);
        }
        
        // Try 11-bit decode  
        if (decodeChannel(raw, 11, &channel, &value) && (channel < 16)) {
            _cs11 |= (1 << channel);
        }
    }
    
    _formatSamples++;
    _frameStats.formatDetectSamples = _formatSamples;
    
    // Need several samples before deciding
    if (_formatSamples < 5) {
        return;
    }
    
    // Check against known channel patterns
    static const uint32_t masks[] = {
        0x3f,   // 6 channels (DX6)
        0x7f,   // 7 channels (DX7)
        0xff,   // 8 channels (DX8)
        0x1ff,  // 9 channels (DX9)
        0x3ff,  // 10 channels (DX10)
        0x7ff,  // 11 channels
        0xfff   // 12 channels
    };
    
    unsigned votes10 = 0;
    unsigned votes11 = 0;
    
    for (unsigned i = 0; i < sizeof(masks)/sizeof(masks[0]); i++) {
        if (_cs10 == masks[i]) votes10++;
        if (_cs11 == masks[i]) votes11++;
    }
    
    if ((votes11 == 1) && (votes10 == 0)) {
        _channelShift = 11;
        _frameStats.is11Bit = true;
        return;
    }
    
    if ((votes10 == 1) && (votes11 == 0)) {
        _channelShift = 10;
        _frameStats.is11Bit = false;
        return;
    }
    
    // Format detection failed, reset and try again
    detectFormat(true);
}

bool SpektrumSatelliteReader::decodeFrame(uint32_t frameTimeMs, const uint8_t frame[DSM_FRAME_SIZE]) {
    _frameStats.validFrames++;
    
    int maxChannel = 0;
    
    // Process each channel in the frame
    for (unsigned i = 0; i < DSM_FRAME_CHANNELS; i++) {
        const uint8_t *dp = &frame[2 + (2 * i)];
        uint16_t raw = (dp[0] << 8) | dp[1];
        unsigned channel, value;
        
        if (!decodeChannel(raw, _channelShift, &channel, &value)) {
            continue;
        }
        
        // Ignore channels out of range
        if (channel >= (unsigned)_numChannels) {
            continue;
        }
        
        // Track max channel for statistics
        if ((int)channel > maxChannel) {
            maxChannel = channel;
        }
        
        // Convert to 1000-2000us PWM range (ArduPilot scaling)
        if (_channelShift == 10) {
            value *= 2;  // Scale 10-bit to 11-bit range
        }
        
        // Spektrum scaling: midpoint 1520us, +/-400us travel
        // Convert to standard 1500us center, 1000-2000us range
        int pwmValue = ((((int)value - 1024) * 1000) / 1700) + 1500;
        pwmValue = constrain(pwmValue, _minValue, _maxValue);
        
        // Store the channel value
        _channelValues[channel] = pwmValue;
    }
    
    _frameStats.detectedChannels = maxChannel + 1;
    
    return true;
}

int SpektrumSatelliteReader::getModeFromChannel4() const {
    int ch4Value = getChannel(3);  // Channel 4 is 0-based index 3
    
    // MODE VALUE mapping with ±40us wiggle room
    if (ch4Value >= 1166 && ch4Value <= 1246) return 1;  // 1206 ± 40
    if (ch4Value >= 1265 && ch4Value <= 1345) return 2;  // 1305 ± 40
    if (ch4Value >= 1362 && ch4Value <= 1442) return 3;  // 1402 ± 40
    if (ch4Value >= 1557 && ch4Value <= 1637) return 4;  // 1597 ± 40
    if (ch4Value >= 1655 && ch4Value <= 1735) return 5;  // 1695 ± 40
    if (ch4Value >= 1753 && ch4Value <= 1833) return 6;  // 1793 ± 40
    
    return 0;  // Invalid/unknown mode
}

SpektrumSatelliteReader::SwitchStates SpektrumSatelliteReader::getSwitchStatesFromChannel6() const {
    int ch6Value = getChannel(5);  // Channel 6 is 0-based index 5
    SwitchStates states = {false, false};
    
    // Switch state mapping with ±40us wiggle room
    if (ch6Value >= 1160 && ch6Value <= 1240) {
        // A=OFF, B=OFF: 1200
        states.switchA = false;
        states.switchB = false;
    }
    else if (ch6Value >= 1350 && ch6Value <= 1450) {
        // A=OFF, B=ON: 1400
        states.switchA = false;
        states.switchB = true;
    }
    else if (ch6Value >= 1550 && ch6Value <= 1650) {
        // A=ON, B=OFF: 1599
        states.switchA = true;
        states.switchB = false;
    }
    else if (ch6Value >= 1750 && ch6Value <= 1850) {
        // A=ON, B=ON: 1800
        states.switchA = true;
        states.switchB = true;
    }
    
    return states;
}

SpektrumSatelliteReader::Channel8State SpektrumSatelliteReader::getChannel8State() const {
    int ch8Value = getChannel(7);  // Channel 8 is 0-based index 7
    
    // Define thresholds for three-way switch (80% detection)
    int lowThreshold = _minValue + (int)(0.2f * (_defaultValue - _minValue));   // 1100us for 1000-1500-2000 range
    int highThreshold = _defaultValue + (int)(0.8f * (_maxValue - _defaultValue)); // 1900us for 1000-1500-2000 range
    
    if (ch8Value <= lowThreshold) {
        return CH8_LOW;
    } else if (ch8Value >= highThreshold) {
        return CH8_HIGH;
    } else if (ch8Value >= (lowThreshold + 50) && ch8Value <= (highThreshold - 50)) {
        return CH8_MIDDLE;  // Add some hysteresis to avoid flutter
    }
    
    return CH8_UNKNOWN;
}

#ifdef ARDUINO_SAMD_VARIANT
// TC3 Interrupt Service Routine for SAMD51
extern "C" void TC3_Handler() {
    SpektrumSatelliteReader::timerInterruptHandler();
}
#endif 
# JumpropeM4

## Overview
JumpropeM4 is a firmware for the Adafruit Feather M4 CAN board to control RS03 motors for use in a jumprope training device. It supports multiple control modes including velocity control and various position control schemes using CRSF input, and can control up to two motors on the same CAN bus.

## Features
- Dual RS03 motor control using CAN bus communication (1 Mbit/s)
- Six distinct operating modes with different control schemes
- CRSF Receiver input
- Real-time motor feedback and status monitoring
- Individual and synchronized motor control options
- Emergency stop and error clearing functionality
- Mechanical zero position setting and calibration
- Status indication via onboard NeoPixel LED
- OLED display showing real-time system status and motor targets
- Active motor feedback reporting with CAN message processing
- Motor direction control (Motor 1 reversed by default)
- Current limiting and motor protection
- Multi-level logging system with detailed error reporting
- Non-blocking initialization with graceful degradation

## Hardware Requirements
- Adafruit Feather M4 CAN board
- Two RS03 motors with CAN interface (configurable IDs)
- ELRS receiver (CRSF compatible)
- SSD1306 OLED display (128x64 pixels, I2C)
- NeoPixel LED (integrated on Feather board)
- Appropriate power supply for motors and controller
- CAN bus termination resistors

## Pin Connections
- **NeoPixel**: Integrated on Feather M4 board
- **ELRS Receiver**: Connected to hardware serial (configurable)
- **CAN Bus**: Connected to RS03 motors via CAN H/L pins
- **I2C Display**: SDA/SCL pins for SSD1306 OLED
- **CAN Control Pins**: Standby and Boost Enable pins for CAN transceiver

## Control Modes

The system supports 6 distinct control modes selected via Channel 4:

| Mode | Name | Description | Primary Control | Secondary Control |
|------|------|-------------|-----------------|-------------------|
| 1 | Velocity Mode | Direct velocity control of both motors | Channel 5 (±20 rad/s) | N/A |
| 2 | Position Mode (Switch) | Binary position control with zero nudging | Switch B (0 or 3.8 rad) | Ch5 (Zero nudge ±0.2 rad) |
| 3 | Position Mode (Multi-Control) | Switch override or analog position control | SwA (-0.1 rad), SwB (3.9 rad), or Ch5 (1.9±2π rad) | Switch priority over Channel 5 |
| 4 | Position Mode (Fixed) | Fixed position targets | Automatic (M1=-4.4, M2=-0.6 rad) | N/A |
| 5 | Position Mode (Individual) | Independent motor control with smoothing | Ch7 (M1 ±7 rad), Ch9 (M2 ±7 rad) | Exponential smoothing (0.15 factor) |
| 6 | Emergency Stop | Robust communication reset | Automatic stop and fault reset (every 3s) | Switch A (Zero) |

## Channel Mapping

### RC Channel Functions

| Channel | Function | Range/Values | Used In Modes | Description |
|---------|----------|--------------|---------------|-------------|
| 4 | Mode Selection | 1-6 | All | Selects operating mode |
| 5 | Velocity/Position/Nudge | 1000-2000μs | 1, 2, 3 | Velocity control (Mode 1), zero nudging (Mode 2), or position offset when no switches pressed (Mode 3) |
| 6 | Switch States | Switch A/B | All | Switch A: Zero setting (Mode 6) or position override (Mode 3), Switch B: Binary position (Mode 2, 3) |
| 7 | Motor 1 Position | 1000-2000μs | 5 | Individual Motor 1 position control (±7 rad) with smoothing |
| 8 | State Tracking | Low/Mid/High | All | Channel 8 state monitoring (display only) |
| 9 | Motor 2 Position | 1000-2000μs | 5 | Individual Motor 2 position control (±7 rad) with smoothing |

### Switch Functions

| Switch | Function | Action | Available In |
|--------|----------|--------|--------------|
| Switch A | Mechanical Zero / Position Override | Mode 6: Sets mechanical zero; Mode 3: Overrides to -0.1 rad | Mode 3, 6 |
| Switch B | Binary Position | Mode 2: OFF=0 rad, ON=3.8 rad (affected by nudging); Mode 3: Overrides to 3.9 rad | Mode 2, 3 |

## Detailed Mode Operation

### Mode 1: Velocity Control
- **Control**: Channel 5 controls velocity of both motors simultaneously
- **Range**: ±20 rad/s with dead zone around center
- **Direction**: Motor 1 reversed, Motor 2 normal
- **Use Case**: Direct speed control for synchronized movement

### Mode 2: Position Control (Switch B + Zero Nudging)
- **Control**: Switch B provides binary position control with adjustable zero offset
- **Positions**: Switch OFF = nudgeable zero position, Switch ON = 3.8 rad - nudgeable zero
- **Zero Nudging**: Channel 5 above 65% or below 35% nudges zero position by ±0.01 rad every 100ms
- **Nudge Range**: ±0.2 radians maximum offset from true zero
- **Motors**: Both motors move to same target position (affected by zero offset)
- **Display**: Shows current nudgeable zero offset (Z: value)
- **Use Case**: Two-position operation with fine zero adjustment capability

### Mode 3: Position Control (Multi-Control)
- **Switch Priority**: Switch inputs override Channel 5 control
- **Switch A**: When pressed, overrides to -0.1 radians
- **Switch B**: When pressed, overrides to 3.9 radians  
- **Channel 5 Control**: When no switches pressed, provides analog position control
- **Base Position**: 1.9 radians (Channel 5 control only)
- **Range**: ±2π radians around base (approximately -4.38 to +8.18 rad)
- **Motors**: Both motors move to same target position
- **Use Case**: Flexible control with quick preset positions or continuous adjustment

### Mode 4: Fixed Position Control
- **Control**: Automatic movement to predetermined positions
- **Targets**: Motor 1 = -4.4 radians, Motor 2 = -0.6 radians
- **Operation**: Motors automatically move to fixed positions
- **Use Case**: Preset position recall

### Mode 5: Individual Motor Control
- **Control**: Independent position control for each motor with smoothing
- **Motor 1**: Channel 7 controls position (±7 radians)
- **Motor 2**: Channel 9 controls position (±7 radians)
- **Smoothing**: Exponential smoothing with 0.15 factor to reduce jerky motions
- **Smoothing Reset**: Automatically resets when entering mode from different mode
- **Use Case**: Asymmetric movements and individual motor testing with smooth motion

### Mode 6: Emergency Stop
- **Function**: Robust communication reset sequence with motor stop
- **Operation**: Automatic every 3 seconds while in mode
- **Reset Sequence**: 
  - Emergency stop (velocity mode, zero velocity)
  - Fault reset and motor re-enable
  - Zero flag configuration (preserves mechanical zero)
  - Active reporting re-establishment
  - Move to stored zero position
  - Reset control variables and nudgeable zero
- **Zero Setting**: Switch A sets NEW mechanical zero position (only works in this mode)
- **Recovery**: Motors re-enabled when exiting mode
- **Use Case**: Emergency situations, communication recovery, and mechanical zero calibration

## Motor Configuration

| Parameter | Motor 1 | Motor 2 | Notes |
|-----------|---------|---------|-------|
| Direction | Reversed | Normal | Motor 1 commands are inverted |
| Position Mode | PP Control | PP Control | 25 rad/s speed, 200 rad/s² accel, 40A current |
| Velocity Mode | Direct | Direct | No additional limits applied |
| Zero Range | -π to π | -π to π | Zero flag set to 1 for both motors |

## LED Status Indicators

| Color | Status | Meaning |
|-------|--------|---------|
| 🟢 Green | Fully Operational | Motors ready + RC signal present |
| 🔵 Cyan | Motors Ready | Motors initialized but no RC signal |
| 🟡 Yellow | Limited Function | Initialization issues, check connections |
| 🔴 Red | Error/No Signal | RC signal lost or system errors |

## OLED Display Information

The 128x64 OLED display shows:
- **Line 1**: "Motor Control" header
- **Line 2**: Current mode (VEL(1), POS(2), POS(3), FIX(4), IND(5), STOP(6))
- **Line 3**: Motor 1 desired position/velocity with units
- **Line 4**: Motor 2 desired position/velocity with units  
- **Line 5**: Switch states (Button A: ON/OFF, Button B: ON/OFF) + Zero nudge offset in Mode 2

## Serial Output

Serial communication at 115200 baud provides:
- Detailed initialization sequence
- Real-time control values and motor targets
- Frame statistics every 5 seconds
- Error messages and warnings
- Control scheme documentation on startup

## System Configuration

Key parameters defined in the firmware:
- **Motors**: Configurable CAN IDs for two RS03 motors
- **Control Limits**: 
  - Velocity: ±20 rad/s maximum
  - Position: Various ranges depending on mode
  - Individual control: ±7 rad range
- **Timing**:
  - Motor updates: Every 20ms  
  - Display updates: Every 150ms
  - Serial output: Every 100ms
  - CAN feedback: Continuous processing
  - Zero nudging: 0.01 rad every 100ms when triggered
  - Mode 6 reset: Every 3 seconds while in mode
- **Dead Zones**: ±50μs around channel center positions
- **Zero Nudging**: 
  - Trigger thresholds: Channel 5 > 65% or < 35%
  - Nudge amount: ±0.01 radians per 100ms
  - Maximum range: ±0.2 radians from true zero
- **Motion Smoothing (Mode 5)**:
  - Exponential smoothing factor: 0.15 (lower = smoother, higher = more responsive)
  - Automatic reset when mode changes
  - Applied to both motor control and display values

## Building and Flashing

This project uses PlatformIO:

```bash
# Install dependencies
pio lib install

# Build firmware  
pio run

# Upload to board
pio run -t upload

# Monitor serial output
pio device monitor
```

## Troubleshooting

### LED Status Diagnosis
- **Red LED**: Check RC receiver connection and power
- **Yellow LED**: Verify motor CAN bus connections and IDs
- **Cyan LED**: RC receiver not detected, motors functional
- **Green LED**: System fully operational

### Common Issues
1. **Motors not responding**: 
   - Verify CAN bus termination
   - Check motor power supply
   - Confirm motor CAN IDs match firmware configuration

2. **No RC control**:
   - Verify ELRS receiver binding
   - Check receiver power and signal connections
   - Monitor serial output for frame statistics
   - Confirm channel 5 armed

3. **Position errors**:
   - Use Switch A to reset mechanical zero
   - Verify motors are properly calibrated
   - Check for mechanical obstructions

### Serial Debugging
Monitor serial output for:
- Frame statistics showing RC signal quality
- Motor initialization sequence results
- Real-time control values and feedback
- Error messages and warnings

## Safety Features
- Emergency stop mode (Mode 6) with robust communication reset sequence
- Graceful degradation when components fail to initialize
- Motor fault detection and clearing with automatic recovery
- Dead zone implementation prevents accidental activation
- Non-blocking initialization prevents system lockup
- Motion smoothing (Mode 5) reduces mechanical stress from jerky movements
- Mechanical zero preservation during communication resets
- Switch-based position overrides for reliable preset positioning

## License
[Specify license information here]
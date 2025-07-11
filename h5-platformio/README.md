# nanoELS H5 - Refactored PlatformIO Version

This is a complete refactoring of the nanoELS H5 Electronic Lead Screw controller, preserving all functionality from the original `h5.ino` while implementing a modern, modular architecture using PlatformIO.

## 🚀 Features

### Core Functionality (Preserved from Original)
- **Multi-axis Control**: X, Y, Z axis control with independent motion planning
- **Enhanced MPG Support**: Velocity-based manual pulse generator control for natural feel
- **G-code Interpretation**: Full G-code support with real-time execution
- **Touch Screen Interface**: Nextion display with modern UI
- **WiFi Connectivity**: Web interface and WebSocket communication
- **PS2 Keyboard Support**: Full keyboard input for direct control
- **Safety Systems**: Comprehensive safety monitoring and emergency stop
- **File System**: G-code storage and management
- **Preferences**: Persistent settings storage

### Enhanced Architecture
- **Modular Design**: Class-based architecture with clear separation of concerns
- **Real-time Tasks**: FreeRTOS task scheduling for optimal performance
- **Thread Safety**: Mutex-protected operations for multi-threaded safety
- **Configuration Management**: Centralized configuration with validation
- **Error Handling**: Comprehensive error detection and recovery
- **Diagnostics**: Built-in self-test and monitoring capabilities

## 📁 Project Structure

```
h5-platformio/
├── include/
│   ├── config/
│   │   ├── hardware_config.h      # Hardware pin definitions and constants
│   │   └── safety_config.h        # Safety system configuration
│   ├── core/
│   │   ├── axis_controller.h      # Individual axis control
│   │   ├── safety_system.h        # Safety monitoring and emergency stop
│   │   ├── spindle_encoder.h      # Spindle encoder handling
│   │   └── motion_planner.h       # Multi-axis motion planning
│   ├── ui/
│   │   └── display_manager.h      # Nextion display interface
│   └── comm/
│       └── communication_manager.h # WiFi, WebSocket, PS2 keyboard
├── src/
│   └── main.cpp                   # Main application entry point
├── platformio.ini                 # PlatformIO configuration
└── README.md                      # This file
```

## 🔧 Hardware Requirements

### ESP32-S3 Development Board
- **Processor**: ESP32-S3 dual-core 240MHz
- **Memory**: 8MB PSRAM, 16MB Flash
- **Connectivity**: WiFi, Bluetooth, USB-C

### Motor Control
- **Stepper Motors**: NEMA 23 or NEMA 17 with closed-loop drivers
- **Power Supply**: 48V for maximum performance
- **Drivers**: Recommended STEPPERONLINE CL57T or similar

### Encoder System
- **Spindle Encoder**: 1200 PPR optical rotary encoder
- **Mounting**: 3D-printed encoder mount and gear
- **Connection**: Quadrature encoder signals

### Display and Input
- **Touch Screen**: Nextion display (recommended 7" or larger)
- **Keyboard**: PS2 keyboard for direct input
- **Interface**: SN74HCT245N level shifters for signal buffering

## 🛠️ Installation and Setup

### Prerequisites
- **PlatformIO**: Install PlatformIO IDE or CLI
- **Arduino Framework**: ESP32 Arduino framework
- **Libraries**: Required libraries are automatically managed by PlatformIO

### Building the Project

1. **Clone the repository**:
   ```bash
   git clone <repository-url>
   cd nanoELS-unleashed/h5-platformio
   ```

2. **Configure hardware settings**:
   Edit `include/config/hardware_config.h` to match your hardware:
   ```cpp
   // WiFi settings
   #define SSID "your-wifi-name"
   #define PASSWORD "your-password"
   
   // Pin assignments
   #define ENC_A 13
   #define ENC_B 14
   // ... other pins
   ```

3. **Build the project**:
   ```bash
   pio run
   ```

4. **Upload to ESP32-S3**:
   ```bash
   pio run --target upload
   ```

5. **Monitor serial output**:
   ```bash
   pio device monitor
   ```

## 🔌 Pin Configuration

### Essential Pins (Required)
```cpp
// Spindle Encoder
#define ENC_A 13
#define ENC_B 14

// Z Axis (Main Lead Screw)
#define Z_ENA 41
#define Z_DIR 42
#define Z_STEP 35
#define Z_PULSE_A 18  // MPG
#define Z_PULSE_B 8   // MPG

// X Axis (Cross Slide)
#define X_ENA 16
#define X_DIR 15
#define X_STEP 7
#define X_PULSE_A 47  // MPG
#define X_PULSE_B 21  // MPG

// PS2 Keyboard
#define KEY_DATA 37
#define KEY_CLOCK 36
```

### Optional Pins
```cpp
// Y Axis (Optional 4th Axis)
#define Y_ENA 1
#define Y_DIR 2
#define Y_STEP 17
#define Y_PULSE_A 45  // MPG
#define Y_PULSE_B 48  // MPG
```

## 🎛️ Operation Modes

### Normal Mode (Gearbox)
- Synchronized motion with spindle
- Automatic pitch control
- Real-time position tracking

### Turn Mode
- Automatic turning operations
- Multi-pass support
- Depth control

### Face Mode
- Automatic facing operations
- Step-over control
- Surface finish optimization

### Thread Mode
- Multi-start threading
- Thread chasing
- Pitch verification

### G-code Mode
- Full G-code interpretation
- File-based programs
- Real-time execution

### Cone Mode
- Automatic cone turning
- Ratio control
- Precision cutting

## 🌐 Web Interface

The system provides a comprehensive web interface accessible via WiFi:

### Features
- **Real-time Status**: Live position, speed, and status updates
- **G-code Management**: Upload, download, and manage G-code files
- **Manual Control**: Direct axis control and parameter adjustment
- **Settings**: Configuration and calibration options
- **Monitoring**: Real-time graphs and diagnostics

### Access
1. Connect to the nanoELS WiFi network
2. Open web browser to `http://nanoels-h5.local` or `192.168.4.1`
3. Use the web interface for control and monitoring

## 🔒 Safety Features

### Emergency Stop System
- **Hardware E-stop**: Immediate stop capability
- **Software E-stop**: Programmatic emergency stop
- **Watchdog Timer**: System health monitoring
- **Fault Detection**: Comprehensive error detection

### Motion Safety
- **Soft Limits**: Programmable travel limits
- **Hard Limits**: Hardware limit switch support
- **Speed Limits**: Maximum speed enforcement
- **Acceleration Limits**: Safe acceleration curves

### System Monitoring
- **Temperature Monitoring**: Component temperature tracking
- **Power Monitoring**: Voltage and current monitoring
- **Communication Health**: Network and device connectivity
- **Encoder Health**: Spindle encoder signal monitoring

## 📊 Enhanced MPG Control

The refactored version includes enhanced Manual Pulse Generator (MPG) control:

### Velocity-Based Movement
- **Natural Feel**: MPG responds to rotation speed, not just pulse count
- **Adaptive Acceleration**: Faster rotation = faster movement
- **Smooth Transitions**: Velocity averaging prevents jerky movement
- **Responsive Control**: Immediate response to user input

### Configuration
```cpp
// MPG velocity control parameters
const float MPG_VELOCITY_SCALE = 0.5;        // Scale factor
const float MPG_ACCELERATION_SCALE = 2.0;    // Acceleration scaling
const int MPG_VELOCITY_SAMPLES = 10;         // Velocity averaging samples
const unsigned long MPG_VELOCITY_TIMEOUT_MS = 100; // Timeout for velocity calculation
```

## 🔧 Configuration

### Hardware Configuration
Edit `include/config/hardware_config.h` for your specific hardware:

```cpp
// Lead screw configuration
#define SCREW_Z_DU 40000    // 4mm lead screw
#define MOTOR_STEPS_Z 800   // Motor steps per revolution

// Speed and acceleration
#define SPEED_START_Z MOTOR_STEPS_Z
#define ACCELERATION_Z 25 * MOTOR_STEPS_Z

// Direction inversion
#define INVERT_Z false      // Invert if needed
#define INVERT_X true       // Invert if needed
```

### Safety Configuration
Edit `include/config/safety_config.h` for safety limits:

```cpp
// Speed limits
#define SAFETY_MAX_SPEED_Z 10000
#define SAFETY_MAX_SPEED_X 10000

// Temperature limits
#define SAFETY_MAX_TEMPERATURE_MOTOR 80
#define SAFETY_MAX_TEMPERATURE_DRIVER 70
```

## 🐛 Troubleshooting

### Common Issues

1. **Build Errors**:
   - Ensure PlatformIO is properly installed
   - Check that all required libraries are available
   - Verify ESP32-S3 board support is installed

2. **Upload Issues**:
   - Check USB connection and drivers
   - Verify correct COM port selection
   - Try different upload speeds

3. **Hardware Issues**:
   - Verify pin connections
   - Check power supply voltage
   - Test encoder signals

4. **WiFi Issues**:
   - Check SSID and password configuration
   - Verify WiFi credentials
   - Check for network conflicts

### Debug Features

The system includes comprehensive debugging:

```cpp
// Enable debug output
#define DEBUG_ESP_PORT Serial
#define DEBUG_ESP_CORE
#define DEBUG_ESP_WIFI
```

### Serial Monitor Commands

```bash
# Monitor with exception decoder
pio device monitor --filters esp32_exception_decoder

# Monitor with timestamps
pio device monitor --filters time
```

## 📈 Performance

### Real-time Performance
- **Motion Control**: 1ms cycle time on Core 1
- **Safety Monitoring**: 10ms cycle time on Core 0
- **Display Updates**: 50ms cycle time (20Hz)
- **Communication**: 20ms cycle time

### Memory Usage
- **Stack Sizes**: Optimized for ESP32-S3 memory
- **Heap Management**: Efficient memory allocation
- **PSRAM Usage**: Leverages external PSRAM for buffers

## 🤝 Contributing

### Development Guidelines
1. **Preserve Functionality**: All original features must be maintained
2. **Safety First**: Safety systems are highest priority
3. **Modular Design**: Keep components loosely coupled
4. **Thread Safety**: Use proper synchronization
5. **Error Handling**: Comprehensive error detection and recovery

### Code Style
- **C++ Standards**: Use modern C++ features
- **Naming**: Clear, descriptive names
- **Documentation**: Comprehensive comments
- **Testing**: Include self-test capabilities

## 📄 License

This project is based on the original nanoELS by Maxim Kachurovskiy. All original work and intellectual property belongs to Maxim. This refactored version maintains full compatibility while adding modern development practices.

## 🙏 Acknowledgments

- **Maxim Kachurovskiy**: Original nanoELS creator
- **ESP32 Community**: Hardware and library support
- **PlatformIO Team**: Development environment
- **Open Source Contributors**: Libraries and tools

---

**⚠️ SAFETY WARNING**: This controls dangerous machinery. Always implement proper emergency stops and safety measures before operating. Use at your own risk.
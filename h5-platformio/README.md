# nanoELS H5 - PlatformIO Edition

**Electronic Lead Screw Controller for Metal Lathes**

This is the enhanced PlatformIO version of the nanoELS H5 lathe controller, featuring a class-based architecture with safety-first design principles.

## Overview

The nanoELS H5 is an ESP32-S3 based controller that provides precise electronic lead screw functionality for metal lathes. This PlatformIO version offers:

- **Class-based architecture** for better maintainability and testing
- **Enhanced safety systems** with hardware emergency stops and watchdogs
- **Professional development environment** with VSCode/Cursor support
- **Comprehensive debugging** capabilities with real-time monitoring
- **Modular design** for easy customization and expansion

## Hardware Requirements

### Core Components
- **ESP32-S3 Development Board** (N16R8 recommended)
- **SN74HCT245N Buffer ICs** (2x) for 5V signal level translation
- **Nextion 5" Display** (NX8048P050 011C Y)
- **PS2 Mini Keyboard** for input
- **Custom PCB** (see `pcb/` directory in original project)

### Mechanical Components
- **600 PPR Optical Encoder** for spindle position
- **NEMA 23 Stepper Motors** with appropriate drivers
- **Lead screws** (4mm pitch recommended)
- **Stepper drivers** (closed-loop recommended, e.g., STEPPERONLINE CL57T)

### Safety Hardware
- **Hardware Emergency Stop Button** (required for safe operation)
- **Hardware Limit Switches** (highly recommended)
- **Proper enclosure** with appropriate ventilation

## Software Features

### Motion Control
- **3-axis control** (Z, X, Y) with precise positioning
- **Multiple operation modes**: Threading, turning, facing, cones, G-code
- **Real-time motion planning** with acceleration control
- **Backlash compensation** and position tracking

### Safety Systems
- **Hardware emergency stop** with immediate response
- **Software watchdog** and position monitoring
- **Motion limits** and collision avoidance
- **Fail-safe operation** with comprehensive error handling

### User Interface
- **Nextion touchscreen** with intuitive interface
- **PS2 keyboard** for precise control
- **WiFi web interface** for remote monitoring and G-code upload
- **Real-time status** display and error reporting

## Development Environment Setup

### Prerequisites
1. **VSCode or Cursor** IDE
2. **PlatformIO extension** installed
3. **Git** for version control

### Installation
```bash
# Clone the repository
git clone https://github.com/your-username/nanoELS-unleashed.git
cd nanoELS-unleashed/h5-platformio

# Open in VSCode/Cursor
code .

# Or open in Cursor
cursor .
```

### First Build
1. Open the project in VSCode/Cursor
2. Install recommended extensions when prompted
3. Wait for PlatformIO to initialize and download dependencies
4. Use `Ctrl+Shift+P` → "PlatformIO: Build" to compile

### Configuration
Edit `include/config/hardware_config.h` to match your hardware setup:

```cpp
// Adjust these values for your specific hardware
#define SPINDLE_ENCODER_PPR     1200    // Your encoder PPR
#define DEFAULT_MOTOR_STEPS_Z   800     // Your motor steps per revolution
#define DEFAULT_SCREW_PITCH_Z   40000   // Your lead screw pitch (deci-microns)
```

## Build Configurations

### Development Build
```bash
pio run -e nanoels_h5
```

### Debug Build (with extensive logging)
```bash
pio run -e nanoels_h5_debug
```

### Release Build (optimized for production)
```bash
pio run -e nanoels_h5_release
```

## Safety Considerations

⚠️ **IMPORTANT SAFETY WARNING** ⚠️

This controller operates dangerous machinery. Always follow these safety guidelines:

### Essential Safety Measures
1. **Install hardware emergency stop** button accessible from all operating positions
2. **Test emergency stop** functionality before each use
3. **Install limit switches** on all axes to prevent crashes
4. **Use proper enclosure** to protect electronics from chips and coolant
5. **Verify all connections** before powering on
6. **Start with low speeds** and test all functions
7. **Never leave machine unattended** during operation

### Safety Features
- Hardware emergency stop with immediate motion halt
- Position limits and soft stops
- Watchdog timer for system reliability
- Error detection and automatic shutdown
- Fail-safe defaults and validation

## Usage

### Basic Operation
1. **Power on** the system and wait for initialization
2. **Home all axes** using the appropriate procedure
3. **Set work coordinates** and material dimensions
4. **Choose operation mode** (turning, threading, etc.)
5. **Set parameters** (speed, feed, depth of cut)
6. **Test with air cuts** before actual machining

### Operation Modes
- **F1 - Gearbox**: Basic electronic gearing
- **F2 - Turning**: Automated turning passes
- **F3 - Facing**: Face turning operations
- **F4 - Cone**: Tapered turning
- **F5 - Parting**: Cut-off operations
- **F6 - Threading**: Internal and external threads
- **F7 - Async**: Time-based movements
- **F8 - Ellipse**: Elliptical turning
- **F9 - G-code**: CNC program execution

### Keyboard Controls
- **Arrow keys**: Manual axis movement
- **+/-**: Adjust feed/speed
- **Enter**: Start operation
- **ESC**: Emergency stop
- **F1-F10**: Select operation modes
- **Numbers**: Parameter entry

## Debugging and Development

### Serial Monitor
```bash
pio device monitor
```

### Debug with Hardware Debugger
1. Connect ESP-PROG or similar debugger
2. Use `F5` to start debugging session
3. Set breakpoints and inspect variables

### Static Analysis
```bash
pio check --verbose
```

### Unit Testing
```bash
pio test
```

## Project Structure

```
h5-platformio/
├── include/
│   ├── config/           # Configuration headers
│   ├── core/            # Core system classes
│   ├── ui/              # User interface components
│   └── comm/            # Communication protocols
├── src/
│   ├── main.cpp         # Main application
│   ├── core/            # Core system implementations
│   ├── ui/              # UI implementations
│   └── comm/            # Communication implementations
├── lib/                 # Local libraries
├── test/                # Unit tests
├── .vscode/             # VSCode configuration
└── platformio.ini       # PlatformIO configuration
```

## Contributing

1. **Fork** the repository
2. **Create feature branch**: `git checkout -b feature/amazing-feature`
3. **Follow coding standards** (see `.vscode/settings.json`)
4. **Add tests** for new functionality
5. **Test thoroughly** with hardware
6. **Submit pull request** with detailed description

## Troubleshooting

### Common Issues

**Build Errors**
- Ensure PlatformIO is up to date
- Check library dependencies in `platformio.ini`
- Verify ESP32 platform is installed

**Upload Failures**
- Check USB cable and driver
- Verify correct COM port
- Try holding BOOT button during upload

**Runtime Issues**
- Check serial monitor for error messages
- Verify hardware connections
- Test with minimal configuration

### Debug Tips
- Use debug build for verbose logging
- Monitor system state via web interface
- Check safety system status first
- Verify encoder signals and motor responses

## License

This project is based on the original nanoELS by Maxim Kachurovskiy. Please see the main repository for licensing information.

## Acknowledgments

- **Maxim Kachurovskiy** - Original nanoELS creator
- **ESP32 Community** - Excellent platform and tools
- **PlatformIO Team** - Outstanding development environment

## Support

For questions and support:
1. Check the [troubleshooting section](#troubleshooting)
2. Review existing [GitHub issues](https://github.com/your-username/nanoELS-unleashed/issues)
3. Create a new issue with detailed description and hardware setup
4. Join the community discussions

---

**Remember: Safety first! Always test thoroughly and use appropriate safety measures when working with machine tools.**
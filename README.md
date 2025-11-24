# MAVLink Data Collector

A Qt-free C++ CLI application for collecting MAVLink vehicle data that mirrors QGroundControl's Fact system architecture.

## Features

- **MAVLink Protocol Support**: Full support for both MAVLink v1 and v2 protocols
- **Fact System**: Complete port of QGroundControl's Fact system for parameter and telemetry management
- **UDP Communication**: Robust UDP connection handling for MAVLink communication
- **Telemetry Collection**: Comprehensive vehicle data collection including:
  - Vehicle attitude and navigation data
  - GPS/GNSS information
  - Battery status
  - RC control data
  - System status and sensors
  - Vibration and temperature data
  - Wind and estimator status
- **Parameter Management**: Full parameter system identical to QGroundControl
- **Real-time Data**: Live telemetry updates with configurable rates
- **Command Line Interface**: Easy-to-use CLI with comprehensive options

## Architecture

The application is built on the same architectural principles as QGroundControl but without Qt dependencies:

### Core Components

1. **Fact System** (`Fact.h`, `FactMetaData.h`, `FactGroup.h`)
   - Type-safe parameter management
   - Metadata-driven validation and conversion
   - Hierarchical data organization

2. **Parameter Manager** (`ParameterManager.h`)
   - Vehicle parameter loading and caching
   - Real-time parameter synchronization
   - Metadata integration

3. **Vehicle Management** (`Vehicle.h`)
   - Vehicle state tracking
   - Message routing and processing
   - Fact group coordination

4. **MAVLink Communication** (`MAVLinkUdpConnection.h`)
   - Dual protocol support (v1/v2)
   - Automatic version detection
   - Connection statistics and monitoring

### Fact Groups

The system includes all major telemetry categories from QGroundControl:

- **VehicleFactGroup**: Core vehicle telemetry (attitude, altitude, speed)
- **VehicleGPSFactGroup**: Primary GPS data
- **VehicleGPS2FactGroup**: Secondary GPS data
- **VehicleBatteryFactGroup**: Battery status and power management
- **VehicleSystemStatusFactGroup**: System health and sensor status
- **VehicleRCFactGroup**: RC control and radio status
- **VehicleVibrationFactGroup**: Vibration monitoring
- **VehicleTemperatureFactGroup**: Temperature sensors
- **VehicleEstimatorStatusFactGroup**: EKF/estimator health
- **VehicleWindFactGroup**: Wind measurements

## Building

### Prerequisites

- CMake 3.20 or higher
- C++20 compatible compiler (GCC 10+, Clang 12+)
- pthread library
- Internet connection (for MAVLink library fetching)

### Build Instructions

```bash
# Clone or navigate to the ported directory
cd /home/fyousfi/Music/qgroundcontrol/ported

# Create build directory
mkdir build
cd build

# Configure with CMake
cmake ..

# Build the project
cmake --build .

# Run the application
./MAVLinkDataCollector --help
```

## Usage

### Basic Usage

```bash
# Connect to default localhost:14550
./MAVLinkDataCollector

# Connect to specific address and port
./MAVLinkDataCollector -a 192.168.1.100 -p 14550

# Enable verbose output
./MAVLinkDataCollector -a 127.0.0.1 -p 14550 -v

# Show connection statistics
./MAVLinkDataCollector -s -v
```

### Command Line Options

- `-h, --help`: Show help message
- `-a, --address <addr>`: UDP target address (default: 127.0.0.1)
- `-p, --port <port>`: UDP target port (default: 14550)
- `-l, --local-port <port>`: Local UDP port (default: auto)
- `-v, --verbose`: Enable verbose output
- `-s, --stats`: Show connection statistics

### Examples

```bash
# Connect to SITL simulation
./MAVLinkDataCollector -a 127.0.0.1 -p 14550 -v

# Connect to real vehicle
./MAVLinkDataCollector -a 192.168.1.100 -p 14550 -s

# Monitor with custom local port
./MAVLinkDataCollector -a 127.0.0.1 -p 14550 -l 14551 -v
```

## Data Output

The application provides real-time telemetry output including:

### Vehicle Information
- System ID, Component ID
- Vehicle type and autopilot type
- Flight mode and system status
- Armed/flying status

### Telemetry Data
- Attitude (roll, pitch, heading, rates)
- Navigation (altitude, speed, climb rate)
- GPS position and satellite data
- Battery voltage, current, and percentage
- RC channels and signal strength
- System sensor health
- Vibration levels
- Temperature readings

### Connection Statistics
- Bytes/packets sent and received
- Packet loss statistics
- MAVLink version detection

## Implementation Details

### MAVLink Integration

The application uses official MAVLink libraries:
- **MAVLink v1**: c_library_v1 from GitHub
- **MAVLink v2**: c_library_v2 from GitHub

Automatic version detection ensures compatibility with both protocols.

### Fact System Port

The Fact system maintains complete compatibility with QGroundControl:
- Identical data types and validation
- Same parameter metadata structure
- Compatible telemetry processing
- Same enum values and constants

### Threading Model

- **Main Thread**: Command processing and user interface
- **Receive Thread**: MAVLink message reception
- **Heartbeat Thread**: Periodic heartbeat transmission
- **Timer Thread**: Fact group update scheduling

## Dependencies

- **MAVLink Libraries**: Automatically fetched via CMake
- **pthread**: POSIX threading support
- **Standard C++ Libraries**: STL containers, algorithms, I/O

## Compatibility

This implementation is designed to be fully compatible with:
- QGroundControl parameter files
- MAVLink-compatible autopilots (PX4, ArduPilot)
- Standard MAVLink ground control stations
- MAVLink SITL simulators

## License

This code follows the same licensing terms as QGroundControl project.

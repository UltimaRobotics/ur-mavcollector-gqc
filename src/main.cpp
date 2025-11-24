#include <iostream>
#include <iomanip>
#include <csignal>
#include <chrono>
#include <thread>
#include <atomic>
#include <memory>
#include <string>
#include <fstream>
#include <sstream>

#include "MAVLinkUdpConnection.h"
#include "Vehicle.h"
#include "ParameterManager.h"

// Global variables for signal handling
std::shared_ptr<MAVLinkUdpConnection> g_connection;
std::shared_ptr<Vehicle> g_vehicle;
std::atomic<bool> g_running(true);
std::ofstream g_dataLog;

// Signal handler for graceful shutdown
void signalHandler(int signal)
{
    std::cout << "\nReceived signal " << signal << ", shutting down..." << std::endl;
    exit(0);
    if (g_dataLog.is_open()) {
        g_dataLog << "\n=== Shutdown at " << std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count() << " ===" << std::endl;
        g_dataLog.close();
    }
    g_running = false;
}

// Data logging functions
void logMessage(const std::string& message)
{
    std::cout << message << std::endl;
    if (g_dataLog.is_open()) {
        g_dataLog << message << std::endl;
        g_dataLog.flush(); // Ensure data is written immediately
    }
}

// Helper function to safely extract variant values
template<typename T>
T safeGetVariant(const FactMetaData::ValueVariant_t& variant, T defaultValue = T{}) {
    try {
        if (std::holds_alternative<T>(variant)) {
            return std::get<T>(variant);
        }
    } catch (const std::bad_variant_access&) {
        // Fall through to default value
    }
    return defaultValue;
}

void logTelemetryData(Vehicle* vehicle)
{
    if (!vehicle) return;

    std::ostringstream oss;
    oss << "\n=== Telemetry Data ===" << std::endl;
    
    // Get timestamp
    auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    oss << "Timestamp: " << timestamp << std::endl;
    
    // Main vehicle facts
    auto roll = vehicle->roll();
    auto pitch = vehicle->pitch();
    auto heading = vehicle->heading();
    auto groundSpeed = vehicle->groundSpeed();
    auto altitudeAMSL = vehicle->altitudeAMSL();
    auto altitudeRelative = vehicle->altitudeRelative();
    auto climbRate = vehicle->climbRate();
    auto throttlePct = vehicle->throttlePct();

    oss << std::fixed << std::setprecision(6);
    if (roll) oss << "Roll: " << safeGetVariant<double>(roll->cookedValue()) << std::endl;
    if (pitch) oss << "Pitch: " << safeGetVariant<double>(pitch->cookedValue()) << std::endl;
    if (heading) oss << "Heading: " << safeGetVariant<double>(heading->cookedValue()) << std::endl;
    if (groundSpeed) oss << "GroundSpeed: " << safeGetVariant<double>(groundSpeed->cookedValue()) << std::endl;
    if (altitudeAMSL) oss << "AltitudeAMSL: " << safeGetVariant<double>(altitudeAMSL->cookedValue()) << std::endl;
    if (altitudeRelative) oss << "AltitudeRelative: " << safeGetVariant<double>(altitudeRelative->cookedValue()) << std::endl;
    if (climbRate) oss << "ClimbRate: " << safeGetVariant<double>(climbRate->cookedValue()) << std::endl;
    if (throttlePct) oss << "Throttle: " << safeGetVariant<uint16_t>(throttlePct->cookedValue()) << std::endl;

    // GPS data
    auto gpsGroup = vehicle->gpsFactGroup();
    if (gpsGroup) {
        auto lat = gpsGroup->getFact("lat");
        auto lon = gpsGroup->getFact("lon");
        auto satellites = gpsGroup->getFact("satellitesVisible");
        auto fixType = gpsGroup->getFact("fixType");
        auto hdop = gpsGroup->getFact("hdop");
        auto vdop = gpsGroup->getFact("vdop");
        auto alt = gpsGroup->getFact("alt");
        auto eph = gpsGroup->getFact("eph");
        auto epv = gpsGroup->getFact("epv");
        
        if (lat && lon) {
            oss << "GPSLat: " << safeGetVariant<int32_t>(lat->cookedValue()) / 1e7 << std::endl;
            oss << "GPSLon: " << safeGetVariant<int32_t>(lon->cookedValue()) / 1e7 << std::endl;
        }
        if (alt) oss << "GPSAlt: " << safeGetVariant<double>(alt->cookedValue()) << std::endl;
        if (satellites) oss << "GPSSatellites: " << static_cast<int>(safeGetVariant<uint8_t>(satellites->cookedValue())) << std::endl;
        if (fixType) oss << "GPSFixType: " << static_cast<int>(safeGetVariant<uint8_t>(fixType->cookedValue())) << std::endl;
        if (hdop) oss << "GPSHDOP: " << safeGetVariant<double>(hdop->cookedValue()) << std::endl;
        if (vdop) oss << "GPSVDOP: " << safeGetVariant<double>(vdop->cookedValue()) << std::endl;
        if (eph) oss << "GPSEPH: " << safeGetVariant<double>(eph->cookedValue()) << std::endl;
        if (epv) oss << "GPSEPV: " << safeGetVariant<double>(epv->cookedValue()) << std::endl;
    }

    // Battery data
    auto batteryGroup = vehicle->batteryFactGroup();
    if (batteryGroup) {
        auto voltage = batteryGroup->getFact("voltage");
        auto current = batteryGroup->getFact("current");
        auto percent = batteryGroup->getFact("percent");
        auto consumed = batteryGroup->getFact("consumed");
        auto remaining = batteryGroup->getFact("remaining");
        auto temperature = batteryGroup->getFact("temperature");
        
        if (voltage) oss << "BatteryVoltage: " << safeGetVariant<float>(voltage->cookedValue()) << std::endl;
        if (current) oss << "BatteryCurrent: " << safeGetVariant<float>(current->cookedValue()) << std::endl;
        if (percent) oss << "BatteryPercent: " << static_cast<int>(safeGetVariant<uint8_t>(percent->cookedValue())) << std::endl;
        if (consumed) oss << "BatteryConsumed: " << safeGetVariant<float>(consumed->cookedValue()) << std::endl;
        if (remaining) oss << "BatteryRemaining: " << safeGetVariant<float>(remaining->cookedValue()) << std::endl;
        if (temperature) oss << "BatteryTemperature: " << safeGetVariant<float>(temperature->cookedValue()) << std::endl;
    }

    // System Status data
    auto systemStatusGroup = vehicle->systemStatusFactGroup();
    if (systemStatusGroup) {
        auto sensorPresent = systemStatusGroup->getFact("sensorPresent");
        auto sensorHealth = systemStatusGroup->getFact("sensorHealth");
        auto sensorErrors = systemStatusGroup->getFact("sensorErrors");
        auto onboardControlSensorsPresent = systemStatusGroup->getFact("onboardControlSensorsPresent");
        auto onboardControlSensorsHealth = systemStatusGroup->getFact("onboardControlSensorsHealth");
        auto onboardControlSensorsErrors = systemStatusGroup->getFact("onboardControlSensorsErrors");
        
        if (sensorPresent) oss << "SensorPresent: 0x" << std::hex << safeGetVariant<uint32_t>(sensorPresent->cookedValue()) << std::dec << std::endl;
        if (sensorHealth) oss << "SensorHealth: 0x" << std::hex << safeGetVariant<uint32_t>(sensorHealth->cookedValue()) << std::dec << std::endl;
        if (sensorErrors) oss << "SensorErrors: 0x" << std::hex << safeGetVariant<uint32_t>(sensorErrors->cookedValue()) << std::dec << std::endl;
        if (onboardControlSensorsPresent) oss << "ControlSensorsPresent: 0x" << std::hex << safeGetVariant<uint32_t>(onboardControlSensorsPresent->cookedValue()) << std::dec << std::endl;
        if (onboardControlSensorsHealth) oss << "ControlSensorsHealth: 0x" << std::hex << safeGetVariant<uint32_t>(onboardControlSensorsHealth->cookedValue()) << std::dec << std::endl;
        if (onboardControlSensorsErrors) oss << "ControlSensorsErrors: 0x" << std::hex << safeGetVariant<uint32_t>(onboardControlSensorsErrors->cookedValue()) << std::dec << std::endl;
    }

    // RC data
    auto rcGroup = vehicle->rcFactGroup();
    if (rcGroup) {
        auto rcRSSI = rcGroup->getFact("rssi");
        auto rcChannelCount = rcGroup->getFact("channelCount");
        auto rcRSSI_DBM = rcGroup->getFact("rssiDbm");
        auto rcRSSIPercent = rcGroup->getFact("rssiPercent");
        
        if (rcRSSI) oss << "RCRSSI: " << static_cast<int>(safeGetVariant<uint8_t>(rcRSSI->cookedValue())) << std::endl;
        if (rcRSSI_DBM) oss << "RCRSSIDBM: " << static_cast<int>(safeGetVariant<int8_t>(rcRSSI_DBM->cookedValue())) << std::endl;
        if (rcRSSIPercent) oss << "RCRSSIPercent: " << static_cast<int>(safeGetVariant<uint8_t>(rcRSSIPercent->cookedValue())) << std::endl;
        if (rcChannelCount) oss << "RCChannels: " << static_cast<int>(safeGetVariant<uint8_t>(rcChannelCount->cookedValue())) << std::endl;
    }

    // Vibration data
    auto vibrationGroup = vehicle->vibrationFactGroup();
    if (vibrationGroup) {
        auto vibrationX = vibrationGroup->getFact("vibrationX");
        auto vibrationY = vibrationGroup->getFact("vibrationY");
        auto vibrationZ = vibrationGroup->getFact("vibrationZ");
        auto clippingX = vibrationGroup->getFact("clippingX");
        auto clippingY = vibrationGroup->getFact("clippingY");
        auto clippingZ = vibrationGroup->getFact("clippingZ");
        
        if (vibrationX) oss << "VibrationX: " << safeGetVariant<float>(vibrationX->cookedValue()) << std::endl;
        if (vibrationY) oss << "VibrationY: " << safeGetVariant<float>(vibrationY->cookedValue()) << std::endl;
        if (vibrationZ) oss << "VibrationZ: " << safeGetVariant<float>(vibrationZ->cookedValue()) << std::endl;
        if (clippingX) oss << "VibrationClippingX: " << static_cast<int>(safeGetVariant<uint8_t>(clippingX->cookedValue())) << std::endl;
        if (clippingY) oss << "VibrationClippingY: " << static_cast<int>(safeGetVariant<uint8_t>(clippingY->cookedValue())) << std::endl;
        if (clippingZ) oss << "VibrationClippingZ: " << static_cast<int>(safeGetVariant<uint8_t>(clippingZ->cookedValue())) << std::endl;
    }

    // Temperature data
    auto temperatureGroup = vehicle->temperatureFactGroup();
    if (temperatureGroup) {
        auto temperature1 = temperatureGroup->getFact("temperature1");
        auto temperature2 = temperatureGroup->getFact("temperature2");
        auto temperature3 = temperatureGroup->getFact("temperature3");
        
        if (temperature1) oss << "Temperature1: " << safeGetVariant<float>(temperature1->cookedValue()) << std::endl;
        if (temperature2) oss << "Temperature2: " << safeGetVariant<float>(temperature2->cookedValue()) << std::endl;
        if (temperature3) oss << "Temperature3: " << safeGetVariant<float>(temperature3->cookedValue()) << std::endl;
    }

    // Estimator Status data
    auto estimatorStatusGroup = vehicle->estimatorStatusFactGroup();
    if (estimatorStatusGroup) {
        auto estimatorFlags = estimatorStatusGroup->getFact("flags");
        auto innovationPosHoriz = estimatorStatusGroup->getFact("innovationPosHoriz");
        auto innovationPosVert = estimatorStatusGroup->getFact("innovationPosVert");
        auto innovationVelHoriz = estimatorStatusGroup->getFact("innovationVelHoriz");
        auto innovationVelVert = estimatorStatusGroup->getFact("innovationVelVert");
        auto innovationMag = estimatorStatusGroup->getFact("innovationMag");
        auto innovationYaw = estimatorStatusGroup->getFact("innovationYaw");
        
        if (estimatorFlags) oss << "EstimatorFlags: 0x" << std::hex << safeGetVariant<uint32_t>(estimatorFlags->cookedValue()) << std::dec << std::endl;
        if (innovationPosHoriz) oss << "InnovationPosHoriz: " << safeGetVariant<float>(innovationPosHoriz->cookedValue()) << std::endl;
        if (innovationPosVert) oss << "InnovationPosVert: " << safeGetVariant<float>(innovationPosVert->cookedValue()) << std::endl;
        if (innovationVelHoriz) oss << "InnovationVelHoriz: " << safeGetVariant<float>(innovationVelHoriz->cookedValue()) << std::endl;
        if (innovationVelVert) oss << "InnovationVelVert: " << safeGetVariant<float>(innovationVelVert->cookedValue()) << std::endl;
        if (innovationMag) oss << "InnovationMag: " << safeGetVariant<float>(innovationMag->cookedValue()) << std::endl;
        if (innovationYaw) oss << "InnovationYaw: " << safeGetVariant<float>(innovationYaw->cookedValue()) << std::endl;
    }

    // Wind data
    auto windGroup = vehicle->windFactGroup();
    if (windGroup) {
        auto windDirection = windGroup->getFact("direction");
        auto windSpeed = windGroup->getFact("speed");
        auto windClimb = windGroup->getFact("climb");
        
        if (windDirection) oss << "WindDirection: " << safeGetVariant<float>(windDirection->cookedValue()) << std::endl;
        if (windSpeed) oss << "WindSpeed: " << safeGetVariant<float>(windSpeed->cookedValue()) << std::endl;
        if (windClimb) oss << "WindClimb: " << safeGetVariant<float>(windClimb->cookedValue()) << std::endl;
    }

    logMessage(oss.str());
}

void logParameters(Vehicle* vehicle)
{
    if (!vehicle) return;

    auto paramManager = vehicle->parameterManager();
    if (!paramManager || !paramManager->parametersReady()) {
        logMessage("=== Parameters ===\nParameters not ready yet.");
        return;
    }

    std::ostringstream oss;
    oss << "\n=== Parameters ===" << std::endl;
    
    // Get parameter count and progress
    auto paramNames = paramManager->parameterNames(1);
    int totalParams = static_cast<int>(paramNames.size());
    double progress = paramManager->loadProgress();
    
    oss << "TotalParameters: " << totalParams << std::endl;
    oss << "LoadingProgress: " << (progress * 100.0) << "%" << std::endl;
    oss << "ParametersReady: " << (paramManager->parametersReady() ? "Yes" : "No") << std::endl;
    
    // Log critical system parameters
    const char* criticalParams[] = {
        "SYS_AUTOSTART", "SYS_ID_THISMAV", "SYSID_MYGCS", "SYS_COMPANION",
        "SYS_LOGFILE", "SYS_NUM_TIMERS", "SYS_HITL", "SYS_HAS_RC",
        "SYS_TYPE", "SYS_AUTOCONFIG_MODE", "SYS_MOT_STARTUP_DELAY",
        "COM_FLTMODE1", "COM_FLTMODE2", "COM_FLTMODE3", "COM_FLTMODE4",
        "COM_FLTMODE5", "COM_FLTMODE6", "COM_FLTMODE7", "COM_FLTMODE8",
        "COM_ARMING_CHECK", "COM_ARM_ECSH", "COM_ARM_HYST", "COM_ARM_WO_GPS",
        "COM_LOW_BAT_ACT", "COM_DL_LOSS_EN", "COM_RC_LOSS_EN",
        "MPC_XY_VEL_MAX", "MPC_Z_VEL_MAX", "MPC_XY_ACC_MAX", "MPC_Z_ACC_MAX",
        "MPC_THR_HOVER", "MPC_MASS", "MPC_TILTMAX_AIR", "MPC_LAND_SPEED",
        "MC_ROLL_P", "MC_PITCH_P", "MC_YAW_P", "MC_ROLLRATE_P", 
        "MC_PITCHRATE_P", "MC_YAWRATE_P", "MC_ROLLRATE_I", "MC_PITCHRATE_I"
    };
    
    for (const char* paramName : criticalParams) {
        auto param = paramManager->getParameter(1, std::string(paramName));
        if (param) {
            oss << paramName << ": " << param->cookedValueString() << std::endl;
        }
    }
    
    oss << "ParameterSummary: Logged " << sizeof(criticalParams)/sizeof(criticalParams[0]) << " critical parameters out of " << totalParams << " total." << std::endl;
    
    logMessage(oss.str());
}

void printVehicleInfo(Vehicle* vehicle)
{
    if (!vehicle) return;

    std::ostringstream oss;
    oss << "\n=== Vehicle Information ===" << std::endl;
    oss << "SystemID: " << static_cast<int>(vehicle->systemId()) << std::endl;
    oss << "ComponentID: " << static_cast<int>(vehicle->componentId()) << std::endl;
    oss << "VehicleType: " << vehicle->vehicleTypeString() << std::endl;
    oss << "Autopilot: " << vehicle->autopilotTypeString() << std::endl;
    oss << "SystemStatus: " << vehicle->systemStatusString() << std::endl;
    oss << "Armed: " << (vehicle->armed() ? "Yes" : "No") << std::endl;
    oss << "Flying: " << (vehicle->flying() ? "Yes" : "No") << std::endl;
    
    logMessage(oss.str());
}

void printConnectionStats(const MAVLinkUdpConnection* connection)
{
    if (!connection) return;

    std::ostringstream oss;
    oss << "\n=== Connection Statistics ===" << std::endl;
    oss << "BytesReceived: " << connection->getBytesReceived() << std::endl;
    oss << "BytesSent: " << connection->getBytesSent() << std::endl;
    oss << "PacketsReceived: " << connection->getPacketsReceived() << std::endl;
    oss << "PacketsSent: " << connection->getPacketsSent() << std::endl;
    oss << "PacketsLost: " << connection->getPacketsLost() << std::endl;
    oss << "MAVLinkVersion: " << connection->getDetectedMavlinkVersion() << std::endl;
    
    logMessage(oss.str());
}

int main(int argc, char* argv[])
{
    std::string targetAddress = "127.0.0.1";
    int targetPort = 14550;
    int localPort = 44003;
    bool verbose = false;
    bool showStats = false;
    bool checkVersion = true;
    bool enableLogging = false;
    std::string logFileName = "mavlink_data.log";
    
    // Parse command line arguments
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "-a" && i + 1 < argc) {
            targetAddress = argv[++i];
        } else if (arg == "-p" && i + 1 < argc) {
            targetPort = std::atoi(argv[++i]);
        } else if (arg == "-l" && i + 1 < argc) {
            localPort = std::atoi(argv[++i]);
        } else if (arg == "-v") {
            verbose = true;
        } else if (arg == "-s") {
            showStats = true;
        } else if (arg == "--log" && i + 1 < argc) {
            enableLogging = true;
            logFileName = argv[++i];
        } else if (arg == "--no-version-check") {
            checkVersion = false;
        } else if (arg == "-h" || arg == "--help") {
            std::cout << "MAVLink Data Collector - Enhanced QGroundControl Parameter Manager\n";
            std::cout << "Usage: " << argv[0] << " [options]\n";
            std::cout << "Options:\n";
            std::cout << "  -a <address>    Target IP address (default: 127.0.0.1)\n";
            std::cout << "  -p <port>       Target port (default: 14550)\n";
            std::cout << "  -l <port>       Local port to listen on (default: 44003)\n";
            std::cout << "  -v              Verbose output\n";
            std::cout << "  -s              Show connection statistics\n";
            std::cout << "  --log <file>    Log all data to file (default: mavlink_data.log)\n";
            std::cout << "  --no-version-check  Skip version check at startup\n";
            std::cout << "  -h, --help      Show this help message\n";
            return 0;
        }
    }

    // Initialize data logging if enabled
    if (enableLogging) {
        g_dataLog.open(logFileName, std::ios::out | std::ios::app);
        if (g_dataLog.is_open()) {
            auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count();
            g_dataLog << "\n=== MAVLink Data Collection Started at " << timestamp << " ===" << std::endl;
            g_dataLog << "Target: " << targetAddress << ":" << targetPort << std::endl;
            g_dataLog << "Local Port: " << localPort << std::endl;
            g_dataLog << "Build: " << __DATE__ << " " << __TIME__ << std::endl;
            g_dataLog << "=============================" << std::endl;
            g_dataLog.flush();
        } else {
            std::cerr << "Warning: Could not open log file " << logFileName << std::endl;
        }
    }

    // Always perform version check at startup
    if (checkVersion) {
        std::ostringstream oss;
        oss << "=== Version Information ===" << std::endl;
        oss << "MAVLink Data Collector v1.0 - Enhanced" << std::endl;
        oss << "Based on QGroundControl Parameter Manager" << std::endl;
        oss << "Build date: " << __DATE__ << " " << __TIME__ << std::endl;
        oss << "MAVLink v2.0 support enabled" << std::endl;
        if (enableLogging) oss << "Data logging enabled: " << logFileName << std::endl;
        oss << "=============================" << std::endl;
        logMessage(oss.str());
    }

    // Set up signal handlers
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    logMessage("MAVLink Data Collector - Enhanced");
    logMessage("=======================");
    logMessage("Connecting to " + targetAddress + ":" + std::to_string(targetPort) + 
               " (local port: " + std::to_string(localPort) + ")");

    // Create connection
    g_connection = std::make_shared<MAVLinkUdpConnection>();
    
    // Set up callbacks
    g_connection->setConnectionChangedCallback([](bool connected) {
        logMessage("Connection " + std::string(connected ? "established" : "lost"));
    });

    g_connection->setMessageReceivedCallback([verbose, enableLogging](const mavlink_message_t& message) {
        std::ostringstream oss;
        if (verbose) {
            oss << "Message: " << static_cast<int>(message.msgid) 
                << " from sys " << static_cast<int>(message.sysid) 
                << " comp " << static_cast<int>(message.compid);
            logMessage(oss.str());
        }
        
        if (enableLogging) {
            // Log every MAVLink message for complete data collection
            if (g_dataLog.is_open()) {
                auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::system_clock::now().time_since_epoch()).count();
                g_dataLog << timestamp << ",MSG," << static_cast<int>(message.msgid) 
                         << "," << static_cast<int>(message.sysid) 
                         << "," << static_cast<int>(message.compid) 
                         << "," << static_cast<int>(message.len) << std::endl;
                g_dataLog.flush();
            }
        }
    });

    // Connect to vehicle
    if (!g_connection->connect(targetAddress, targetPort, localPort)) {
        std::cerr << "Failed to connect to " << targetAddress << ":" << targetPort << std::endl;
        if (g_dataLog.is_open()) {
            g_dataLog << "ERROR: Failed to connect" << std::endl;
            g_dataLog.close();
        }
        return 1;
    }

    // Create vehicle
    g_vehicle = std::make_shared<Vehicle>(g_connection.get());
    
    // Set up parameter manager callbacks
    auto paramManager = g_vehicle->parameterManager();
    if (paramManager) {
        paramManager->setParametersReadyCallback([](bool ready) {
            logMessage("Parameters " + std::string(ready ? "ready!" : "not ready"));
        });
        
        paramManager->setLoadProgressCallback([](double progress) {
            logMessage("Parameter loading progress: " + std::to_string(progress * 100.0) + "%");
        });
        
        // Start parameter loading
        logMessage("Starting parameter loading...");
        paramManager->refreshAllParameters();
    }

    // Set up vehicle change callback
    g_vehicle->setVehicleChangedCallback([](const Vehicle* vehicle) {
        logMessage("Vehicle state changed");
    });

    logMessage("Connected! Waiting for vehicle data...");
    logMessage("Press Ctrl+C to stop.");

    // Main loop with comprehensive data collection
    auto lastPrintTime = std::chrono::steady_clock::now();
    auto printInterval = std::chrono::seconds(1);
    
    while (g_running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        // Print and log telemetry data periodically
        auto now = std::chrono::steady_clock::now();
        if (now - lastPrintTime >= printInterval) {
            logParameters(g_vehicle.get());
            logTelemetryData(g_vehicle.get());
            
            if (showStats) {
                printConnectionStats(g_connection.get());
            }
            
            lastPrintTime = now;
        }
    }

    // Final statistics
    logMessage("\n=== Final Statistics ===");
    printConnectionStats(g_connection.get());
    printVehicleInfo(g_vehicle.get());

    logMessage("\nShutting down...");
    
    // Cleanup
    g_vehicle.reset();
    g_connection.reset();
    
    if (g_dataLog.is_open()) {
        auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        g_dataLog << "\n=== MAVLink Data Collection Ended at " << timestamp << " ===" << std::endl;
        g_dataLog.close();
    }
    
    logMessage("Done.");
    return 0;
}

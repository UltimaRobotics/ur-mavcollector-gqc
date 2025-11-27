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
#include <variant>
#include <cstdlib>  // For std::getenv

#include "JsonConfig.h"
#include "MAVLinkUdpConnection.h"
#include "Vehicle.h"
#include "ParameterManager.h"
#include "FactMetaData.h"

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
        // System-level battery information
        auto mavlinkVersion = batteryGroup->getFact("mavlinkVersion");
        auto batteryCount = batteryGroup->getFact("batteryCount");
        auto batterySystemType = batteryGroup->getFact("batterySystemType");
        
        if (mavlinkVersion) {
            auto versionVal = safeGetVariant<uint8_t>(mavlinkVersion->cookedValue());
            std::string versionStr = (versionVal == 1) ? "v1.0" : 
                                    (versionVal == 2) ? "v2.0" : "Unknown";
            oss << "BatteryMAVLinkVersion: " << versionStr << std::endl;
        }
        if (batteryCount) {
            auto countVal = safeGetVariant<uint8_t>(batteryCount->cookedValue());
            std::string countStr = (countVal == 255) ? "None" : 
                                  (countVal == 1) ? "1 Battery" : 
                                  (countVal == 2) ? "2 Batteries" : 
                                  std::to_string(countVal) + " Batteries";
            oss << "BatteryCount: " << countStr << std::endl;
        }
        if (batterySystemType) {
            auto typeVal = safeGetVariant<uint8_t>(batterySystemType->cookedValue());
            std::string typeStr = (typeVal == 0) ? "No Battery" : 
                                 (typeVal == 1) ? "Basic System" : 
                                 (typeVal == 2) ? "Enhanced System" : 
                                 (typeVal == 3) ? "Dual Battery System" : 
                                 (typeVal == 4) ? "Multi-Battery System" : "Unknown";
            oss << "BatterySystemType: " << typeStr << std::endl;
        }
        
        // Individual battery metrics
        auto voltage = batteryGroup->getFact("voltage");
        auto current = batteryGroup->getFact("current");
        auto percent = batteryGroup->getFact("percent");
        auto consumed = batteryGroup->getFact("consumed");
        auto remaining = batteryGroup->getFact("remaining");
        auto temperature = batteryGroup->getFact("temperature");
        auto id = batteryGroup->getFact("id");
        auto function = batteryGroup->getFact("function");
        auto type = batteryGroup->getFact("type");
        auto timeRemaining = batteryGroup->getFact("timeRemaining");
        auto chargeState = batteryGroup->getFact("chargeState");
        auto mode = batteryGroup->getFact("mode");
        auto faultBitmask = batteryGroup->getFact("faultBitmask");
        auto cellCount = batteryGroup->getFact("cellCount");
        
        if (voltage) oss << "BatteryVoltage: " << safeGetVariant<float>(voltage->cookedValue()) << " V" << std::endl;
        if (current) oss << "BatteryCurrent: " << safeGetVariant<float>(current->cookedValue()) << " A" << std::endl;
        if (percent) {
            auto percentVal = safeGetVariant<uint8_t>(percent->cookedValue());
            if (percentVal != 255) {
                oss << "BatteryPercent: " << static_cast<int>(percentVal) << "%" << std::endl;
            } else {
                oss << "BatteryPercent: N/A" << std::endl;
            }
        }
        if (consumed) oss << "BatteryConsumed: " << safeGetVariant<float>(consumed->cookedValue()) << " Ah" << std::endl;
        if (remaining) oss << "BatteryRemaining: " << safeGetVariant<float>(remaining->cookedValue()) << " Ah" << std::endl;
        if (temperature) oss << "BatteryTemperature: " << safeGetVariant<float>(temperature->cookedValue()) << " °C" << std::endl;
        if (id) {
            auto idVal = safeGetVariant<uint8_t>(id->cookedValue());
            if (idVal != 255) {
                oss << "BatteryID: " << static_cast<int>(idVal) << std::endl;
            }
        }
        if (function) oss << "BatteryFunction: " << static_cast<int>(safeGetVariant<uint8_t>(function->cookedValue())) << std::endl;
        if (type) oss << "BatteryType: " << static_cast<int>(safeGetVariant<uint8_t>(type->cookedValue())) << std::endl;
        if (timeRemaining) {
            auto timeVal = safeGetVariant<uint32_t>(timeRemaining->cookedValue());
            if (timeVal != UINT32_MAX) {
                oss << "BatteryTimeRemaining: " << timeVal << " s" << std::endl;
            }
        }
        if (chargeState) {
            auto chargeVal = safeGetVariant<uint8_t>(chargeState->cookedValue());
            if (chargeVal != UINT8_MAX) {
                oss << "BatteryChargeState: " << static_cast<int>(chargeVal) << std::endl;
            }
        }
        if (mode) {
            auto modeVal = safeGetVariant<uint8_t>(mode->cookedValue());
            if (modeVal != UINT8_MAX) {
                oss << "BatteryMode: " << static_cast<int>(modeVal) << std::endl;
            }
        }
        if (faultBitmask) {
            auto faultVal = safeGetVariant<uint32_t>(faultBitmask->cookedValue());
            if (faultVal != UINT32_MAX) {
                oss << "BatteryFaultBitmask: 0x" << std::hex << faultVal << std::dec << std::endl;
            }
        }
        if (cellCount) {
            auto cellVal = safeGetVariant<uint16_t>(cellCount->cookedValue());
            if (cellVal > 0) {
                oss << "BatteryCellCount: " << cellVal << std::endl;
            }
        }
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
    
    // Autopilot version information
    oss << "\n=== Autopilot Version Information ===" << std::endl;
    oss << "Board Identification: " << vehicle->boardIdentification() << std::endl;
    oss << "Board Class: " << vehicle->boardClass() << std::endl;
    oss << "Board Name: " << vehicle->boardName() << std::endl;
    oss << "Vendor ID: " << vehicle->vendorId() << std::endl;
    oss << "Product ID: " << vehicle->productId() << std::endl;
    oss << "UID (Serial): " << vehicle->uid() << std::endl;
    oss << "Board Version: " << vehicle->boardVersion() << std::endl;
    oss << "Flight SW Version: " << vehicle->flightSwVersionString() << std::endl;
    oss << "Middleware SW Version: " << vehicle->middlewareSwVersionString() << std::endl;
    oss << "OS SW Version: " << vehicle->osSwVersionString() << std::endl;
    oss << "Flight Custom Version (Git): " << vehicle->flightCustomVersionString() << std::endl;
    oss << "Capabilities: " << vehicle->capabilitiesString() << std::endl;
    
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
    
    // Health monitoring statistics
    oss << "\n=== Health Monitoring ===" << std::endl;
    oss << "HealthCheckEnabled: " << (connection->isHealthCheckEnabled() ? "Yes" : "No") << std::endl;
    oss << "ConnectionHealthy: " << (connection->isConnectionHealthy() ? "Yes" : "No") << std::endl;
    oss << "TimeSinceLastMessage: " << connection->getTimeSinceLastMessage() << "ms" << std::endl;
    oss << "ConnectionTimeout: " << connection->getConnectionTimeout() << "ms" << std::endl;
    oss << "AutoRestartEnabled: " << (connection->isAutoRestartEnabled() ? "Yes" : "No") << std::endl;
    oss << "AutoRestartDelay: " << connection->getAutoRestartDelay() << "ms" << std::endl;
    oss << "RestartCount: " << connection->getRestartCount() << std::endl;
    
    logMessage(oss.str());
}

int main(int argc, char* argv[])
{
    std::string configFilePath = "config.json";
    
    // Parse command line arguments
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "-pkg_config" && i + 1 < argc) {
            configFilePath = argv[++i];
        } else if (arg == "-h" || arg == "--help") {
            std::cout << "MAVLink Data Collector - Enhanced QGroundControl Parameter Manager\n";
            std::cout << "Usage: " << argv[0] << " -pkg_config <config_file>\n";
            std::cout << "Options:\n";
            std::cout << "  -pkg_config <file>    Path to JSON configuration file (default: config.json)\n";
            std::cout << "  -h, --help           Show this help message\n";
            std::cout << "\nJSON Configuration Format:\n";
            std::cout << "  {\n";
            std::cout << "    \"target_address\": \"127.0.0.1\",\n";
            std::cout << "    \"port\": 14550,\n";
            std::cout << "    \"system_id\": 255,\n";
            std::cout << "    \"component_id\": 158,\n";
            std::cout << "    \"health_check_enabled\": true,\n";
            std::cout << "    \"auto_restart_enabled\": true,\n";
            std::cout << "    \"connection_timeout_ms\": 5000,\n";
            std::cout << "    \"restart_delay_ms\": 1000,\n";
            std::cout << "    \"verbose_logging\": false,\n";
            std::cout << "    \"show_statistics\": true,\n";
            std::cout << "    \"enable_data_logging\": false,\n";
            std::cout << "    \"log_file_path\": \"mavlink_data.log\",\n";
            std::cout << "    \"version_check_enabled\": true,\n";
            std::cout << "    \"auto_version_detection\": true\n";
            std::cout << "  }\n";
            return 0;
        }
    }
    
    // Load JSON configuration
    JsonConfig config;
    if (!config.loadFromFile(configFilePath)) {
        std::cerr << "Failed to load configuration from: " << configFilePath << std::endl;
        return 1;
    }
    
    // Extract configuration values
    std::string targetAddress = config.getString("target_address", "127.0.0.1");
    int port = config.getInt("port", 14550);
    uint8_t systemId = static_cast<uint8_t>(config.getInt("system_id", 255));
    uint8_t componentId = static_cast<uint8_t>(config.getInt("component_id", 158));
    
    bool verbose = config.getBool("verbose_logging", false);
    bool showStats = config.getBool("show_statistics", false);
    bool checkVersion = config.getBool("version_check_enabled", true);
    bool enableLogging = config.getBool("enable_data_logging", false);
    std::string logFileName = config.getString("log_file_path", "mavlink_data.log");
    
    // Health monitoring options
    bool enableHealthCheck = config.getBool("health_check_enabled", true);
    bool enableAutoRestart = config.getBool("auto_restart_enabled", true);
    uint32_t connectionTimeout = static_cast<uint32_t>(config.getInt("connection_timeout_ms", 5000));
    uint32_t restartDelay = static_cast<uint32_t>(config.getInt("restart_delay_ms", 1000));
    
    // Print loaded configuration
    std::cout << "=== Configuration Loaded from: " << configFilePath << " ===" << std::endl;
    config.printConfig();

    // Initialize data logging if enabled
    if (enableLogging) {
        g_dataLog.open(logFileName, std::ios::out | std::ios::app);
        if (g_dataLog.is_open()) {
            auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count();
            g_dataLog << "\n=== MAVLink Data Collection Started at " << timestamp << " ===" << std::endl;
            g_dataLog << "Config File: " << configFilePath << std::endl;
            g_dataLog << "Target: " << targetAddress << ":" << port << std::endl;
            g_dataLog << "System ID: " << static_cast<int>(systemId) << std::endl;
            g_dataLog << "Component ID: " << static_cast<int>(componentId) << std::endl;
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
        oss << "Configuration: " << configFilePath << std::endl;
        oss << "=============================" << std::endl;
        logMessage(oss.str());
    }

    // Set up signal handlers
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    logMessage("MAVLink Data Collector - Enhanced");
    logMessage("=======================");
    
    // Show configuration summary
    std::ostringstream configInfo;
    configInfo << "Configuration Summary:" << std::endl;
    configInfo << "Target: " << targetAddress << ":" << port << std::endl;
    configInfo << "System ID: " << static_cast<int>(systemId) << std::endl;
    configInfo << "Component ID: " << static_cast<int>(componentId) << std::endl;
    logMessage(configInfo.str());

    // Create connection
    g_connection = std::make_shared<MAVLinkUdpConnection>();
    
    // Configure system ID and component ID from JSON configuration
    g_connection->setSystemId(systemId);
    g_connection->setComponentId(componentId);
    
    // Configure connection health monitoring
    g_connection->enableConnectionHealthCheck(enableHealthCheck);
    g_connection->setAutoRestartEnabled(enableAutoRestart);
    g_connection->setConnectionTimeout(connectionTimeout);
    g_connection->setAutoRestartDelay(restartDelay);
    
    if (enableHealthCheck) {
        logMessage("Connection health monitoring enabled:");
        std::ostringstream healthConfig;
        healthConfig << "- Timeout: " << connectionTimeout << "ms" << std::endl;
        healthConfig << "- Auto-restart: " << (enableAutoRestart ? "enabled" : "disabled") << std::endl;
        if (enableAutoRestart) {
            healthConfig << "- Restart delay: " << restartDelay << "ms" << std::endl;
        }
        logMessage(healthConfig.str());
    } else {
        logMessage("Connection health monitoring disabled");
    }
    
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
    if (!g_connection->connect(targetAddress, port)) {
        std::cerr << "Failed to connect to " << targetAddress << ":" << port << std::endl;
        if (g_dataLog.is_open()) {
            g_dataLog << "ERROR: Failed to connect" << std::endl;
            g_dataLog.close();
        }
        return 1;
    }

    // Create vehicle with system ID from configuration
    g_vehicle = std::make_shared<Vehicle>(g_connection.get());
    
    // Configure system ID and component ID from JSON configuration
    g_vehicle->setSystemId(systemId);
    g_vehicle->setComponentId(componentId);
    
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

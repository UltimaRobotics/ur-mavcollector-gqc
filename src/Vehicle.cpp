#include "Vehicle.h"
#include "VehicleFactGroup.h"
#include "VehicleGPSFactGroup.h"
#include "VehicleGPS2FactGroup.h"
#include "VehicleBatteryFactGroup.h"
#include "VehicleSystemStatusFactGroup.h"
#include "VehicleRCFactGroup.h"
#include "VehicleVibrationFactGroup.h"
#include "VehicleTemperatureFactGroup.h"
#include "VehicleEstimatorStatusFactGroup.h"
#include "VehicleWindFactGroup.h"
#include "ParameterManager.h"
#include <iostream>
#include <iomanip>

Vehicle::Vehicle(MAVLinkUdpConnection* connection)
    : FactGroup(100) // Update every 100ms
    , _connection(connection)
{
    _initializeFactGroups();
    
    if (_connection) {
        _connection->setVehicle(this);
    }
    
    _parameterManager = std::make_shared<ParameterManager>(this);
}

Vehicle::~Vehicle()
{
    if (_connection) {
        _connection->setVehicle(nullptr);
    }
}

bool Vehicle::sendMessage(const mavlink_message_t &message)
{
    if (_connection) {
        return _connection->sendMessage(message, _systemId, _componentId);
    }
    return false;
}

bool Vehicle::sendCommand(uint16_t command, uint8_t confirmation, 
                         float param1, float param2, float param3, 
                         float param4, float param5, float param6, float param7)
{
    mavlink_message_t message;
    mavlink_msg_command_long_pack(255, MAV_COMP_ID_MISSIONPLANNER, &message,
                                 _systemId, _componentId, command, confirmation,
                                 param1, param2, param3, param4, param5, param6, param7);
    
    return sendMessage(message);
}

bool Vehicle::armed() const
{
    return (_baseMode & MAV_MODE_FLAG_SAFETY_ARMED) != 0;
}

bool Vehicle::flying() const
{
    // Simple heuristic: if armed and altitude > 1m, consider flying
    auto altitudeFact = altitudeRelative();
    if (altitudeFact && armed()) {
        double altitude = std::get<double>(altitudeFact->cookedValue());
        return altitude > 1.0;
    }
    return false;
}

std::string Vehicle::flightMode() const
{
    // This would need to be implemented based on autopilot-specific custom modes
    // For now, return a generic string based on base mode
    if (_baseMode & MAV_MODE_FLAG_CUSTOM_MODE_ENABLED) {
        return "Custom";
    } else if (_baseMode & MAV_MODE_FLAG_STABILIZE_ENABLED) {
        return "Stabilize";
    } else if (_baseMode & MAV_MODE_FLAG_GUIDED_ENABLED) {
        return "Guided";
    } else if (_baseMode & MAV_MODE_FLAG_AUTO_ENABLED) {
        return "Auto";
    } else if (_baseMode & MAV_MODE_FLAG_MANUAL_INPUT_ENABLED) {
        return "Manual";
    }
    return "Unknown";
}

std::string Vehicle::vehicleTypeString() const
{
    switch (_vehicleType) {
        case MAV_TYPE_QUADROTOR:
            return "Quadcopter";
        case MAV_TYPE_COAXIAL:
            return "Coaxial";
        case MAV_TYPE_HELICOPTER:
            return "Helicopter";
        case MAV_TYPE_FIXED_WING:
            return "Fixed Wing";
        case MAV_TYPE_GROUND_ROVER:
            return "Ground Rover";
        case MAV_TYPE_SURFACE_BOAT:
            return "Boat";
        case MAV_TYPE_SUBMARINE:
            return "Submarine";
        case MAV_TYPE_HEXAROTOR:
            return "Hexacopter";
        case MAV_TYPE_OCTOROTOR:
            return "Octocopter";
        case MAV_TYPE_TRICOPTER:
            return "Tricopter";
        default:
            return "Unknown";
    }
}

std::string Vehicle::autopilotTypeString() const
{
    switch (_autopilotType) {
        case MAV_AUTOPILOT_PX4:
            return "PX4";
        case MAV_AUTOPILOT_ARDUPILOTMEGA:
            return "ArduPilot";
        case MAV_AUTOPILOT_GENERIC:
            return "Generic";
        default:
            return "Unknown";
    }
}

std::string Vehicle::systemStatusString() const
{
    switch (_systemStatus) {
        case MAV_STATE_UNINIT:
            return "Uninitialized";
        case MAV_STATE_BOOT:
            return "Booting";
        case MAV_STATE_CALIBRATING:
            return "Calibrating";
        case MAV_STATE_STANDBY:
            return "Standby";
        case MAV_STATE_ACTIVE:
            return "Active";
        case MAV_STATE_CRITICAL:
            return "Critical";
        case MAV_STATE_EMERGENCY:
            return "Emergency";
        case MAV_STATE_POWEROFF:
            return "Powering Off";
        case MAV_STATE_FLIGHT_TERMINATION:
            return "Flight Termination";
        default:
            return "Unknown";
    }
}

void Vehicle::handleMessage(const mavlink_message_t &message)
{
    switch (message.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT:
            _handleHeartbeat(message);
            break;
            
        case MAVLINK_MSG_ID_STATUSTEXT:
            _handleStatustext(message);
            break;
            
        case MAVLINK_MSG_ID_COMMAND_ACK:
            _handleCommandAck(message);
            break;
            
        default:
            // Let fact groups handle the message
            _updateAllValues();
            
            // Forward to fact groups
            for (const auto& pair : _nameToFactGroupMap) {
                const auto& factGroup = pair.second;
                if (factGroup) {
                    factGroup->handleMessage(this, message);
                }
            }
            
            // Forward to parameter manager
            if (_parameterManager) {
                _parameterManager->mavlinkMessageReceived(message);
            }
            break;
    }
}

void Vehicle::_initializeFactGroups()
{
    // Create main vehicle fact group
    auto vehicleFactGroup = std::make_shared<VehicleFactGroup>();
    _addFactGroup(vehicleFactGroup, "vehicle");
    
    // Create GPS fact group
    auto gpsFactGroup = std::make_shared<VehicleGPSFactGroup>();
    _addFactGroup(gpsFactGroup, "gps");
    
    // Create GPS2 fact group
    auto gps2FactGroup = std::make_shared<VehicleGPS2FactGroup>();
    _addFactGroup(gps2FactGroup, "gps2");
    
    // Create battery fact group
    auto batteryFactGroup = std::make_shared<VehicleBatteryFactGroup>();
    _addFactGroup(batteryFactGroup, "battery");
    
    // Create system status fact group
    auto systemStatusFactGroup = std::make_shared<VehicleSystemStatusFactGroup>();
    _addFactGroup(systemStatusFactGroup, "systemStatus");
    
    // Create RC fact group
    auto rcFactGroup = std::make_shared<VehicleRCFactGroup>();
    _addFactGroup(rcFactGroup, "rc");
    
    // Create vibration fact group
    auto vibrationFactGroup = std::make_shared<VehicleVibrationFactGroup>();
    _addFactGroup(vibrationFactGroup, "vibration");
    
    // Create temperature fact group
    auto temperatureFactGroup = std::make_shared<VehicleTemperatureFactGroup>();
    _addFactGroup(temperatureFactGroup, "temperature");
    
    // Create estimator status fact group
    auto estimatorStatusFactGroup = std::make_shared<VehicleEstimatorStatusFactGroup>();
    _addFactGroup(estimatorStatusFactGroup, "estimatorStatus");
    
    // Create wind fact group
    auto windFactGroup = std::make_shared<VehicleWindFactGroup>();
    _addFactGroup(windFactGroup, "wind");
    
    // Add main vehicle facts to this group for easy access
    auto mainFacts = vehicleFactGroup->factNames();
    for (const auto& factName : mainFacts) {
        auto fact = vehicleFactGroup->getFact(factName);
        if (fact) {
            _addFact(fact, factName);
        }
    }
}

void Vehicle::_handleHeartbeat(const mavlink_message_t &message)
{
    mavlink_heartbeat_t heartbeat;
    mavlink_msg_heartbeat_decode(&message, &heartbeat);
    
    std::cout << "Received heartbeat from sysid=" << static_cast<int>(message.sysid) 
              << " compid=" << static_cast<int>(message.compid) 
              << " type=" << static_cast<int>(heartbeat.type)
              << " autopilot=" << static_cast<int>(heartbeat.autopilot) << std::endl;
    
    uint8_t oldSystemId = _systemId;
    uint8_t oldComponentId = _componentId;
    uint8_t oldVehicleType = _vehicleType;
    uint8_t oldAutopilotType = _autopilotType;
    uint8_t oldBaseMode = _baseMode;
    uint32_t oldCustomMode = _customMode;
    uint8_t oldSystemStatus = _systemStatus;
    
    _systemId = message.sysid;
    _componentId = message.compid;
    _vehicleType = heartbeat.type;
    _autopilotType = heartbeat.autopilot;
    _baseMode = heartbeat.base_mode;
    _customMode = heartbeat.custom_mode;
    _systemStatus = heartbeat.system_status;
    _mavlinkVersion = heartbeat.mavlink_version;
    
    // Request parameters on first heartbeat
    static bool firstHeartbeat = true;
    if (firstHeartbeat && _parameterManager) {
        std::cout << "First heartbeat received, requesting parameters..." << std::endl;
        _parameterManager->refreshAllParameters();
        firstHeartbeat = false;
    }
    
    // Check if anything changed
    if (oldSystemId != _systemId || oldComponentId != _componentId ||
        oldVehicleType != _vehicleType || oldAutopilotType != _autopilotType ||
        oldBaseMode != _baseMode || oldCustomMode != _customMode ||
        oldSystemStatus != _systemStatus) {
        _notifyVehicleChanged();
    }
    
    _lastHeartbeatTime = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
}

void Vehicle::_handleStatustext(const mavlink_message_t &message)
{
    mavlink_statustext_t statustext;
    mavlink_msg_statustext_decode(&message, &statustext);
    
    std::string text(reinterpret_cast<char*>(statustext.text));
    std::string severity;
    
    switch (statustext.severity) {
        case MAV_SEVERITY_EMERGENCY:
            severity = "EMERGENCY";
            break;
        case MAV_SEVERITY_ALERT:
            severity = "ALERT";
            break;
        case MAV_SEVERITY_CRITICAL:
            severity = "CRITICAL";
            break;
        case MAV_SEVERITY_ERROR:
            severity = "ERROR";
            break;
        case MAV_SEVERITY_WARNING:
            severity = "WARNING";
            break;
        case MAV_SEVERITY_NOTICE:
            severity = "NOTICE";
            break;
        case MAV_SEVERITY_INFO:
            severity = "INFO";
            break;
        case MAV_SEVERITY_DEBUG:
            severity = "DEBUG";
            break;
        default:
            severity = "UNKNOWN";
            break;
    }
    
    if (_vehicleTextMessageCallback) {
        _vehicleTextMessageCallback(this, severity, text);
    }
}

void Vehicle::_handleCommandAck(const mavlink_message_t &message)
{
    mavlink_command_ack_t commandAck;
    mavlink_msg_command_ack_decode(&message, &commandAck);
    
    // Handle command acknowledgment
    // This could be expanded to track pending commands
}

void Vehicle::_notifyVehicleChanged()
{
    if (_vehicleChangedCallback) {
        _vehicleChangedCallback(this);
    }
}

#include "VehicleBatteryFactGroup.h"
#include "Vehicle.h"
#include "Fact.h"
#include "FactGroup.h"

// Include only the specific MAVLink headers needed to avoid conflicts
#include "../thirdparty/c_library_v2/common/mavlink.h"
#include "../thirdparty/c_library_v2/common/mavlink_msg_battery_status.h"

#include "../thirdparty/c_library_v2/common/mavlink_msg_battery_info.h"
#include "../thirdparty/c_library_v2/common/mavlink_msg_smart_battery_info.h"
#include "../thirdparty/c_library_v2/ardupilotmega/mavlink_msg_battery2.h"

#include <cstring>     // For memcpy in packed structure handling
#include <cstdint>     // For standard integer types
#include <memory>      // For std::shared_ptr used in fact creation and battery groups
#include <iostream>    // For debug output logging (std::cout)
#include <string>      // For string operations in battery info fields

VehicleBatteryFactGroup::VehicleBatteryFactGroup(bool ignoreCamelCase)
    : FactGroup(500, ignoreCamelCase) // Update every 500ms
{
    // Add all basic battery facts for primary battery
    _addFact(std::make_shared<Fact>(0, "voltage", FactMetaData::valueTypeFloat));
    _addFact(std::make_shared<Fact>(0, "current", FactMetaData::valueTypeFloat));
    _addFact(std::make_shared<Fact>(0, "consumed", FactMetaData::valueTypeFloat));
    _addFact(std::make_shared<Fact>(0, "remaining", FactMetaData::valueTypeFloat));
    _addFact(std::make_shared<Fact>(0, "percent", FactMetaData::valueTypeUint8));
    _addFact(std::make_shared<Fact>(0, "temperature", FactMetaData::valueTypeFloat));
    _addFact(std::make_shared<Fact>(0, "id", FactMetaData::valueTypeUint8));
    _addFact(std::make_shared<Fact>(0, "function", FactMetaData::valueTypeUint8));
    _addFact(std::make_shared<Fact>(0, "type", FactMetaData::valueTypeUint8));
    _addFact(std::make_shared<Fact>(0, "timeRemaining", FactMetaData::valueTypeUint32));
    _addFact(std::make_shared<Fact>(0, "chargeState", FactMetaData::valueTypeUint8));
    _addFact(std::make_shared<Fact>(0, "mode", FactMetaData::valueTypeUint8));
    _addFact(std::make_shared<Fact>(0, "faultBitmask", FactMetaData::valueTypeUint32));
    _addFact(std::make_shared<Fact>(0, "cellCount", FactMetaData::valueTypeUint16));
    
    // Add enhanced battery info facts (from BATTERY_INFO)
    _addFact(std::make_shared<Fact>(0, "dischargeMinVoltage", FactMetaData::valueTypeFloat));
    _addFact(std::make_shared<Fact>(0, "chargingMinVoltage", FactMetaData::valueTypeFloat));
    _addFact(std::make_shared<Fact>(0, "restingMinVoltage", FactMetaData::valueTypeFloat));
    _addFact(std::make_shared<Fact>(0, "chargingMaxVoltage", FactMetaData::valueTypeFloat));
    _addFact(std::make_shared<Fact>(0, "chargingMaxCurrent", FactMetaData::valueTypeFloat));
    _addFact(std::make_shared<Fact>(0, "nominalVoltage", FactMetaData::valueTypeFloat));
    _addFact(std::make_shared<Fact>(0, "dischargeMaxCurrent", FactMetaData::valueTypeFloat));
    _addFact(std::make_shared<Fact>(0, "dischargeMaxBurstCurrent", FactMetaData::valueTypeFloat));
    _addFact(std::make_shared<Fact>(0, "designCapacity", FactMetaData::valueTypeFloat));
    _addFact(std::make_shared<Fact>(0, "fullChargeCapacity", FactMetaData::valueTypeFloat));
    _addFact(std::make_shared<Fact>(0, "cycleCount", FactMetaData::valueTypeUint16));
    _addFact(std::make_shared<Fact>(0, "weight", FactMetaData::valueTypeUint16));
    _addFact(std::make_shared<Fact>(0, "stateOfHealth", FactMetaData::valueTypeUint8));
    _addFact(std::make_shared<Fact>(0, "cellsInSeries", FactMetaData::valueTypeUint8));
    _addFact(std::make_shared<Fact>(0, "manufactureDate", FactMetaData::valueTypeString));
    _addFact(std::make_shared<Fact>(0, "serialNumber", FactMetaData::valueTypeString));
    _addFact(std::make_shared<Fact>(0, "batteryName", FactMetaData::valueTypeString));
    
    // Add smart battery info facts (from SMART_BATTERY_INFO)
    _addFact(std::make_shared<Fact>(0, "capacityFullSpecification", FactMetaData::valueTypeInt32));
    _addFact(std::make_shared<Fact>(0, "capacityFull", FactMetaData::valueTypeInt32));
    _addFact(std::make_shared<Fact>(0, "smartSerialNumber", FactMetaData::valueTypeString));
    _addFact(std::make_shared<Fact>(0, "deviceName", FactMetaData::valueTypeString));
    _addFact(std::make_shared<Fact>(0, "smartManufactureDate", FactMetaData::valueTypeString));
    
    // Add cell voltage tracking facts
    _addFact(std::make_shared<Fact>(0, "cellCountDetected", FactMetaData::valueTypeUint8));
    
    // Add system-level battery facts for dynamic detection
    _addFact(std::make_shared<Fact>(0, "mavlinkVersion", FactMetaData::valueTypeUint8));
    _addFact(std::make_shared<Fact>(0, "batteryCount", FactMetaData::valueTypeUint8));
    _addFact(std::make_shared<Fact>(0, "batterySystemType", FactMetaData::valueTypeUint8));
    
    // Initialize system facts to unknown state
    mavlinkVersion()->setRawValue(static_cast<uint8_t>(0));
    batteryCount()->setRawValue(static_cast<uint8_t>(0));
    batterySystemType()->setRawValue(static_cast<uint8_t>(0));
    cellCountDetected()->setRawValue(static_cast<uint8_t>(0));
}

void VehicleBatteryFactGroup::handleMessage([[maybe_unused]] Vehicle *vehicle, const mavlink_message_t &message)
{
    // Comprehensive battery message handling
    switch (message.msgid) {
        // Standard battery status messages
        case MAVLINK_MSG_ID_BATTERY_STATUS:
            std::cout << "[BATTERY] Handling BATTERY_STATUS message (ID: " << static_cast<int>(message.msgid) << ")" << std::endl;
            _handleBatteryStatus(message);
            break;
            
        // Battery info messages
        case MAVLINK_MSG_ID_BATTERY_INFO:
            std::cout << "[BATTERY] Handling BATTERY_INFO message (ID: " << static_cast<int>(message.msgid) << ")" << std::endl;
            _handleBatteryInfo(message);
            break;
            
        // Smart battery info messages
        case MAVLINK_MSG_ID_SMART_BATTERY_INFO:
            std::cout << "[BATTERY] Handling SMART_BATTERY_INFO message (ID: " << static_cast<int>(message.msgid) << ")" << std::endl;
            _handleSmartBatteryInfo(message);
            break;
            
        // ArduPilot battery2 messages
        case MAVLINK_MSG_ID_BATTERY2:
            std::cout << "[BATTERY] Handling BATTERY2 message (ID: " << static_cast<int>(message.msgid) << ")" << std::endl;
            _handleBattery2(message);
            break;
            
        // System status (for basic battery info fallback)
        case MAVLINK_MSG_ID_SYS_STATUS:
            _handleSysStatus(message);
            break;
            
        default:
            // Not a battery message, ignore
            break;
    }
}

// Enhanced battery status handler with comprehensive decoding
void VehicleBatteryFactGroup::_handleBatteryStatus(const mavlink_message_t &message)
{
    // Use standard battery status for compatibility
    mavlink_battery_status_t batteryStatus;
    mavlink_msg_battery_status_decode(&message, &batteryStatus);
    
    std::cout << "[BATTERY] === BATTERY_STATUS Message Decoded ===" << std::endl;
    std::cout << "[BATTERY] Battery ID: " << static_cast<int>(batteryStatus.id) << std::endl;
    std::cout << "[BATTERY] Function: " << static_cast<int>(batteryStatus.battery_function) << std::endl;
    std::cout << "[BATTERY] Type: " << static_cast<int>(batteryStatus.type) << std::endl;
    
    // Process cell voltages (up to 10 cells for v1, up to 14 for v2)
    uint8_t detectedCellCount = 0;
    
    // For v2 messages, we have extended voltages
    if (message.len > MAVLINK_MSG_ID_BATTERY_STATUS_MIN_LEN) {
        // Try to access extended voltages if available (v2+)
        const uint16_t* voltagesExt = nullptr;
        uint8_t extCount = 0;
        
        // Check if we can safely access extended fields
        if (message.len >= sizeof(mavlink_battery_status_t)) {
            // For v2 messages, try to access extended fields safely
            // Note: This is a simplified approach - proper v2 handling would need proper structure access
            voltagesExt = nullptr; // Will be implemented with proper v2 structure access
            extCount = 0;
        }
        
        _processCellVoltages(batteryStatus.voltages, 10, voltagesExt, extCount);
    } else {
        // v1 message - only main voltages
        _processCellVoltages(batteryStatus.voltages, 10, nullptr, 0);
    }
    
    // Calculate total voltage from first cell or sum of all cells
    float batteryVoltage = NAN;
    if (batteryStatus.voltages[0] != UINT16_MAX) {
        batteryVoltage = static_cast<float>(batteryStatus.voltages[0]) / 1000.0f;
        detectedCellCount = 1;
        
        // Count additional valid cells
        for (int i = 1; i < 10; i++) {
            if (batteryStatus.voltages[i] != UINT16_MAX) {
                detectedCellCount++;
            }
        }
    }
    
    // Extract current
    float batteryCurrent = NAN;
    if (batteryStatus.current_battery != -1) {
        batteryCurrent = static_cast<float>(batteryStatus.current_battery) / 100.0f;
    }
    
    // Extract consumed charge
    float consumed = NAN;
    if (batteryStatus.current_consumed != -1) {
        consumed = static_cast<float>(batteryStatus.current_consumed);
    }
    
    // Extract remaining percentage
    float remaining = NAN;
    uint8_t percent = 0;
    if (batteryStatus.battery_remaining != -1) {
        percent = static_cast<uint8_t>(batteryStatus.battery_remaining);
        remaining = static_cast<float>(batteryStatus.battery_remaining);
    }
    
    // Extract temperature
    float temperature = NAN;
    if (batteryStatus.temperature != INT16_MAX) {
        temperature = static_cast<float>(batteryStatus.temperature) / 100.0f;
    }
    
    // Extract additional v2 fields (if available)
    uint32_t timeRemaining = 0;
    uint8_t chargeState = 0;
    uint8_t mode = 0;
    uint32_t faultBitmask = 0;
    
    // For now, use basic v1 fields - v2 extended fields would need proper structure access
    timeRemaining = 0;
    chargeState = 0;
    mode = 0;
    faultBitmask = 0;
    
    std::cout << "[BATTERY] Voltage: " << (batteryVoltage != NAN ? std::to_string(batteryVoltage) : "N/A") << "V" << std::endl;
    std::cout << "[BATTERY] Current: " << (batteryCurrent != NAN ? std::to_string(batteryCurrent) : "N/A") << "A" << std::endl;
    std::cout << "[BATTERY] Percent: " << static_cast<int>(percent) << "%" << std::endl;
    std::cout << "[BATTERY] Temperature: " << (temperature != NAN ? std::to_string(temperature) : "N/A") << "°C" << std::endl;
    std::cout << "[BATTERY] Cell Count: " << static_cast<int>(detectedCellCount) << std::endl;
    std::cout << "[BATTERY] =============================================" << std::endl;
    
    // Update all battery facts
    _updateBatteryFacts(batteryStatus.id, batteryVoltage, batteryCurrent, consumed, remaining, percent,
                       temperature, batteryStatus.battery_function, batteryStatus.type,
                       timeRemaining, chargeState, mode, faultBitmask, detectedCellCount);
    
    // Update detected cell count
    cellCountDetected()->setRawValue(detectedCellCount);
}

// Access cell voltage by index
std::shared_ptr<Fact> VehicleBatteryFactGroup::cellVoltage(uint8_t cellIndex)
{
    std::string factName = "cell" + std::to_string(cellIndex + 1) + "Voltage";
    return getFact(factName);
}

// Battery info handler for comprehensive battery information
void VehicleBatteryFactGroup::_handleBatteryInfo(const mavlink_message_t &message)
{
    mavlink_battery_info_t batteryInfo;
    mavlink_msg_battery_info_decode(&message, &batteryInfo);
    
    std::cout << "[BATTERY] === BATTERY_INFO Message Decoded ===" << std::endl;
    std::cout << "[BATTERY] Battery ID: " << static_cast<int>(batteryInfo.id) << std::endl;
    std::cout << "[BATTERY] Design Capacity: " << batteryInfo.design_capacity << "Ah" << std::endl;
    std::cout << "[BATTERY] Full Charge Capacity: " << batteryInfo.full_charge_capacity << "Ah" << std::endl;
    std::cout << "[BATTERY] Cycle Count: " << batteryInfo.cycle_count << std::endl;
    std::cout << "[BATTERY] State of Health: " << static_cast<int>(batteryInfo.state_of_health) << "%" << std::endl;
    std::cout << "[BATTERY] Serial Number: " << std::string(batteryInfo.serial_number) << std::endl;
    std::cout << "[BATTERY] Name: " << std::string(batteryInfo.name) << std::endl;
    std::cout << "[BATTERY] ======================================" << std::endl;
    
    // Update battery info facts
    _updateBatteryInfoFacts(batteryInfo.id, batteryInfo);
}

// Smart battery info handler for smart battery data
void VehicleBatteryFactGroup::_handleSmartBatteryInfo(const mavlink_message_t &message)
{
    mavlink_smart_battery_info_t smartBatteryInfo;
    mavlink_msg_smart_battery_info_decode(&message, &smartBatteryInfo);
    
    std::cout << "[BATTERY] === SMART_BATTERY_INFO Message Decoded ===" << std::endl;
    std::cout << "[BATTERY] Battery ID: " << static_cast<int>(smartBatteryInfo.id) << std::endl;
    std::cout << "[BATTERY] Capacity Full Spec: " << smartBatteryInfo.capacity_full_specification << "mAh" << std::endl;
    std::cout << "[BATTERY] Capacity Full: " << smartBatteryInfo.capacity_full << "mAh" << std::endl;
    std::cout << "[BATTERY] Cycle Count: " << smartBatteryInfo.cycle_count << std::endl;
    std::cout << "[BATTERY] Device Name: " << std::string(smartBatteryInfo.device_name) << std::endl;
    std::cout << "[BATTERY] Serial Number: " << std::string(smartBatteryInfo.serial_number) << std::endl;
    std::cout << "[BATTERY] ==========================================" << std::endl;
    
    // Update smart battery info facts
    _updateSmartBatteryInfoFacts(smartBatteryInfo.id, smartBatteryInfo);
}

// Battery2 handler for ArduPilot battery2 messages
void VehicleBatteryFactGroup::_handleBattery2(const mavlink_message_t &message)
{
    mavlink_battery2_t battery2;
    mavlink_msg_battery2_decode(&message, &battery2);
    
    std::cout << "[BATTERY] === BATTERY2 Message Decoded ===" << std::endl;
    std::cout << "[BATTERY] Voltage: " << static_cast<float>(battery2.voltage) / 1000.0f << "V" << std::endl;
    std::cout << "[BATTERY] Current: " << (battery2.current_battery != -1 ? 
             std::to_string(static_cast<float>(battery2.current_battery) / 100.0f) : "N/A") << "A" << std::endl;
    std::cout << "[BATTERY] ==============================" << std::endl;
    
    // Update battery2 facts (for battery ID 1 typically)
    _updateBattery2Facts(1, battery2);
}

// Process cell voltages from battery status messages
void VehicleBatteryFactGroup::_processCellVoltages(const uint16_t* voltages, uint8_t voltageCount, 
                                                  const uint16_t* voltagesExt, uint8_t extCount)
{
    uint8_t totalCells = 0;
    
    // Count valid cells in main voltage array
    for (uint8_t i = 0; i < voltageCount; i++) {
        if (voltages[i] != UINT16_MAX) {
            totalCells++;
        }
    }
    
    // Count valid cells in extended voltage array
    for (uint8_t i = 0; i < extCount; i++) {
        if (voltagesExt[i] != 0) { // Extended cells use 0 for invalid
            totalCells++;
        }
    }
    
    // Update max cell count and add facts if needed
    if (totalCells > _maxCellCount) {
        _maxCellCount = totalCells;
        _addCellVoltageFacts(totalCells);
        cellCountDetected()->setRawValue(totalCells);
    }
    
    // Update individual cell voltages
    for (uint8_t i = 0; i < voltageCount && i < _maxCellCount; i++) {
        if (voltages[i] != UINT16_MAX) {
            std::string factName = "cell" + std::to_string(i + 1) + "Voltage";
            auto fact = getFact(factName);
            if (fact) {
                fact->setRawValue(static_cast<float>(voltages[i]) / 1000.0f);
            }
        }
    }
    
    // Update extended cell voltages
    for (uint8_t i = 0; i < extCount && (voltageCount + i) < _maxCellCount; i++) {
        if (voltagesExt[i] != 0) {
            std::string factName = "cell" + std::to_string(voltageCount + i + 1) + "Voltage";
            auto fact = getFact(factName);
            if (fact) {
                fact->setRawValue(static_cast<float>(voltagesExt[i]) / 1000.0f);
            }
        }
    }
}

// Add cell voltage facts for the specified number of cells
void VehicleBatteryFactGroup::_addCellVoltageFacts(uint8_t cellCount)
{
    for (uint8_t i = 0; i < cellCount; i++) {
        std::string factName = "cell" + std::to_string(i + 1) + "Voltage";
        
        // Only add if it doesn't already exist
        if (!getFact(factName)) {
            _addFact(std::make_shared<Fact>(0, factName, FactMetaData::valueTypeFloat));
        }
    }
}

// Update battery info facts from BATTERY_INFO message
void VehicleBatteryFactGroup::_updateBatteryInfoFacts(uint8_t batteryId, const __mavlink_battery_info_t& info)
{
    // Create a typed copy for easier access
    mavlink_battery_info_t batteryInfo;
    std::memcpy(&batteryInfo, &info, sizeof(batteryInfo));
    
    // Update basic info facts
    dischargeMinVoltage()->setRawValue(batteryInfo.discharge_minimum_voltage);
    chargingMinVoltage()->setRawValue(batteryInfo.charging_minimum_voltage);
    restingMinVoltage()->setRawValue(batteryInfo.resting_minimum_voltage);
    chargingMaxVoltage()->setRawValue(batteryInfo.charging_maximum_voltage);
    chargingMaxCurrent()->setRawValue(batteryInfo.charging_maximum_current);
    nominalVoltage()->setRawValue(batteryInfo.nominal_voltage);
    dischargeMaxCurrent()->setRawValue(batteryInfo.discharge_maximum_current);
    dischargeMaxBurstCurrent()->setRawValue(batteryInfo.discharge_maximum_burst_current);
    designCapacity()->setRawValue(batteryInfo.design_capacity);
    fullChargeCapacity()->setRawValue(batteryInfo.full_charge_capacity);
    cycleCount()->setRawValue(batteryInfo.cycle_count);
    weight()->setRawValue(batteryInfo.weight);
    stateOfHealth()->setRawValue(batteryInfo.state_of_health);
    cellsInSeries()->setRawValue(batteryInfo.cells_in_series);
    
    // Update string fields
    manufactureDate()->setRawValue(std::string(batteryInfo.manufacture_date));
    serialNumber()->setRawValue(std::string(batteryInfo.serial_number));
    batteryName()->setRawValue(std::string(batteryInfo.name));
    
    // Update battery ID
    id()->setRawValue(batteryId);
}

// Update smart battery info facts from SMART_BATTERY_INFO message
void VehicleBatteryFactGroup::_updateSmartBatteryInfoFacts(uint8_t batteryId, const __mavlink_smart_battery_info_t& info)
{
    // Create a typed copy for easier access (avoid packed field issues)
    mavlink_smart_battery_info_t smartBatteryInfo;
    std::memcpy(&smartBatteryInfo, &info, sizeof(smartBatteryInfo));
    
    // Update smart battery specific facts (use local copies to avoid packed field issues)
    int32_t capacityFullSpec = smartBatteryInfo.capacity_full_specification;
    int32_t capacityFullVal = smartBatteryInfo.capacity_full;
    uint16_t cycleCount = smartBatteryInfo.cycle_count;
    uint16_t weight = smartBatteryInfo.weight;
    
    capacityFullSpecification()->setRawValue(capacityFullSpec);
    capacityFull()->setRawValue(capacityFullVal);
    
    // Update string fields
    smartSerialNumber()->setRawValue(std::string(smartBatteryInfo.serial_number));
    deviceName()->setRawValue(std::string(smartBatteryInfo.device_name));
    smartManufactureDate()->setRawValue(std::string(smartBatteryInfo.manufacture_date));
    
    // Update additional info
    this->cycleCount()->setRawValue(cycleCount);
    this->weight()->setRawValue(weight);
    
    // Update battery ID
    id()->setRawValue(batteryId);
}

// Update battery2 facts from BATTERY2 message
void VehicleBatteryFactGroup::_updateBattery2Facts(uint8_t batteryId, const __mavlink_battery2_t& battery2)
{
    // Create a typed copy for easier access (avoid packed field issues)
    mavlink_battery2_t battery2Typed;
    std::memcpy(&battery2Typed, &battery2, sizeof(battery2Typed));
    
    // Update voltage and current
    float batteryVoltage = static_cast<float>(battery2Typed.voltage) / 1000.0f;
    float batteryCurrent = (battery2Typed.current_battery != -1) ? 
                   static_cast<float>(battery2Typed.current_battery) / 100.0f : NAN;
    
    voltage()->setRawValue(batteryVoltage);
    current()->setRawValue(batteryCurrent);
    
    // Update battery ID
    id()->setRawValue(batteryId);
}

// System status handler for basic battery fallback
void VehicleBatteryFactGroup::_handleSysStatus(const mavlink_message_t &message)
{
    mavlink_sys_status_t sysStatus;
    mavlink_msg_sys_status_decode(&message, &sysStatus);
    
    std::cout << "[BATTERY] SYS_STATUS - Voltage: " << (sysStatus.voltage_battery/1000.0f) 
              << "V, Current: " << (sysStatus.current_battery/100.0f) 
              << "A, Percent: " << (int)sysStatus.battery_remaining << "%" << std::endl;
    
    // Update basic battery facts from SYS_STATUS
    voltage()->setRawValue(static_cast<float>(sysStatus.voltage_battery / 1000.0f));
    current()->setRawValue(static_cast<float>(sysStatus.current_battery / 100.0f));
    percent()->setRawValue(static_cast<uint8_t>(sysStatus.battery_remaining));
    
    _setTelemetryAvailable(true);
}

// Helper method to update battery facts for any battery ID
void VehicleBatteryFactGroup::_updateBatteryFacts(uint8_t batteryId, float voltageVal, float currentVal, 
                                                 float consumedVal, float remainingVal, uint8_t percentVal, 
                                                 float temperatureVal, uint8_t functionVal, uint8_t typeVal,
                                                 uint32_t timeRemainingVal, uint8_t chargeStateVal,
                                                 uint8_t modeVal, uint32_t faultBitmaskVal, uint16_t cellCountVal)
{
    // Update primary battery facts
    voltage()->setRawValue(voltageVal);
    current()->setRawValue(currentVal);
    consumed()->setRawValue(consumedVal);
    remaining()->setRawValue(remainingVal);
    percent()->setRawValue(percentVal);
    temperature()->setRawValue(temperatureVal);
    id()->setRawValue(batteryId);
    function()->setRawValue(functionVal);
    type()->setRawValue(typeVal);
    timeRemaining()->setRawValue(timeRemainingVal);
    chargeState()->setRawValue(chargeStateVal);
    mode()->setRawValue(modeVal);
    faultBitmask()->setRawValue(faultBitmaskVal);
    cellCount()->setRawValue(cellCountVal);
    
    _setTelemetryAvailable(true);
}

// Get or create fact group for specific battery ID
std::shared_ptr<FactGroup> VehicleBatteryFactGroup::_getOrCreateBatteryGroup(uint8_t batteryId)
{
    if (_batteryFactGroups.find(batteryId) == _batteryFactGroups.end()) {
        auto newGroup = std::make_shared<FactGroup>(500);
        _addBatterySpecificFacts(batteryId);
        _batteryFactGroups[batteryId] = newGroup;
    }
    return _batteryFactGroups[batteryId];
}

// Add battery-specific facts for multi-battery support
void VehicleBatteryFactGroup::_addBatterySpecificFacts([[maybe_unused]] uint8_t batteryId)
{
    // Implementation for multi-battery support
    // For now, we rely on the primary facts
}

// Public methods to access specific battery data
std::shared_ptr<Fact> VehicleBatteryFactGroup::voltage(uint8_t batteryId)
{
    if (batteryId == 0) {
        return voltage();
    }
    auto group = _getOrCreateBatteryGroup(batteryId);
    return group ? group->getFact("voltage") : nullptr;
}

std::shared_ptr<Fact> VehicleBatteryFactGroup::current(uint8_t batteryId)
{
    if (batteryId == 0) {
        return current();
    }
    auto group = _getOrCreateBatteryGroup(batteryId);
    return group ? group->getFact("current") : nullptr;
}

std::shared_ptr<Fact> VehicleBatteryFactGroup::percent(uint8_t batteryId)
{
    if (batteryId == 0) {
        return percent();
    }
    auto group = _getOrCreateBatteryGroup(batteryId);
    return group ? group->getFact("percent") : nullptr;
}

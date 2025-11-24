#include "VehicleBatteryFactGroup.h"
#include "Vehicle.h"
#include <mavlink/v2.0/common/mavlink.h>

VehicleBatteryFactGroup::VehicleBatteryFactGroup(bool ignoreCamelCase)
    : FactGroup(500, ignoreCamelCase) // Update every 500ms
{
    // Add all battery facts
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
}

void VehicleBatteryFactGroup::handleMessage(Vehicle *vehicle, const mavlink_message_t &message)
{
    switch (message.msgid) {
        case MAVLINK_MSG_ID_BATTERY_STATUS:
            _handleBatteryStatus(message);
            break;
            
        case MAVLINK_MSG_ID_SYS_STATUS:
            _handleSysStatus(message);
            break;
    }
}

void VehicleBatteryFactGroup::_handleBatteryStatus(const mavlink_message_t &message)
{
    mavlink_battery_status_t batteryStatus;
    mavlink_msg_battery_status_decode(&message, &batteryStatus);
    
    voltage()->setRawValue(static_cast<float>(batteryStatus.voltages[0] / 1000.0f)); // Convert mV to V
    current()->setRawValue(static_cast<float>(batteryStatus.current_battery / 100.0f)); // Convert to A
    consumed()->setRawValue(static_cast<float>(batteryStatus.current_consumed / 1000.0f)); // Convert mAh to Ah
    remaining()->setRawValue(static_cast<float>(batteryStatus.energy_consumed / 1000.0f)); // Convert mWh to Wh
    percent()->setRawValue(static_cast<uint8_t>(batteryStatus.battery_remaining));
    temperature()->setRawValue(static_cast<float>(batteryStatus.temperature / 100.0f)); // Convert to Celsius
    id()->setRawValue(static_cast<uint8_t>(batteryStatus.id));
    function()->setRawValue(static_cast<uint8_t>(batteryStatus.battery_function));
    type()->setRawValue(static_cast<uint8_t>(batteryStatus.type));
    // time_remaining and charge_state fields are not available in this MAVLink version
    // timeRemaining()->setRawValue(static_cast<uint32_t>(batteryStatus.time_remaining));
    // chargeState()->setRawValue(static_cast<uint8_t>(batteryStatus.charge_state));
    
    _setTelemetryAvailable(true);
}

void VehicleBatteryFactGroup::_handleSysStatus(const mavlink_message_t &message)
{
    mavlink_sys_status_t sysStatus;
    mavlink_msg_sys_status_decode(&message, &sysStatus);
    
    // Use the voltage() method instead of voltageBattery()
    voltage()->setRawValue(static_cast<float>(sysStatus.voltage_battery / 1000.0f)); // Convert mV to V
    current()->setRawValue(static_cast<float>(sysStatus.current_battery / 100.0f)); // Convert to A
    percent()->setRawValue(static_cast<uint8_t>(sysStatus.battery_remaining));
    
    _setTelemetryAvailable(true);
}

#include "VehicleGPSFactGroup.h"
#include "Vehicle.h"
#include <mavlink/v2.0/common/mavlink.h>
#include <iomanip>

VehicleGPSFactGroup::VehicleGPSFactGroup(bool ignoreCamelCase)
    : FactGroup(1000, ignoreCamelCase) // Update every 1 second
{
    // Add all GPS facts
    _addFact(std::make_shared<Fact>(0, "lat", FactMetaData::valueTypeInt32));
    _addFact(std::make_shared<Fact>(0, "lon", FactMetaData::valueTypeInt32));
    _addFact(std::make_shared<Fact>(0, "alt", FactMetaData::valueTypeInt32));
    _addFact(std::make_shared<Fact>(0, "altEllipsoid", FactMetaData::valueTypeInt32));
    _addFact(std::make_shared<Fact>(0, "hdop", FactMetaData::valueTypeUint16));
    _addFact(std::make_shared<Fact>(0, "vdop", FactMetaData::valueTypeUint16));
    _addFact(std::make_shared<Fact>(0, "course", FactMetaData::valueTypeFloat));
    _addFact(std::make_shared<Fact>(0, "groundSpeed", FactMetaData::valueTypeFloat));
    _addFact(std::make_shared<Fact>(0, "count", FactMetaData::valueTypeUint8));
    _addFact(std::make_shared<Fact>(0, "lock", FactMetaData::valueTypeUint8));
    _addFact(std::make_shared<Fact>(0, "satellitesVisible", FactMetaData::valueTypeUint8));
    _addFact(std::make_shared<Fact>(0, "utcDate", FactMetaData::valueTypeUint32));
    _addFact(std::make_shared<Fact>(0, "utcTime", FactMetaData::valueTypeUint32));
    _addFact(std::make_shared<Fact>(0, "timeUtc", FactMetaData::valueTypeUint64));
    _addFact(std::make_shared<Fact>(0, "fixType", FactMetaData::valueTypeUint8));
    _addFact(std::make_shared<Fact>(0, "eph", FactMetaData::valueTypeFloat));
    _addFact(std::make_shared<Fact>(0, "epv", FactMetaData::valueTypeFloat));
    _addFact(std::make_shared<Fact>(0, "heading", FactMetaData::valueTypeFloat));
    _addFact(std::make_shared<Fact>(0, "speedAccuracy", FactMetaData::valueTypeFloat));
    _addFact(std::make_shared<Fact>(0, "horizAccuracy", FactMetaData::valueTypeFloat));
    _addFact(std::make_shared<Fact>(0, "vertAccuracy", FactMetaData::valueTypeFloat));
    _addFact(std::make_shared<Fact>(0, "yaw", FactMetaData::valueTypeFloat));
    _addFact(std::make_shared<Fact>(0, "yawAccuracy", FactMetaData::valueTypeFloat));
}

void VehicleGPSFactGroup::handleMessage(Vehicle *vehicle, const mavlink_message_t &message)
{
    switch (message.msgid) {
        case MAVLINK_MSG_ID_GPS_RAW_INT:
            _handleGPSRawInt(message);
            break;
            
        case MAVLINK_MSG_ID_GPS2_RAW:
            _handleGPS2Raw(message);
            break;
            
        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
            _handleGlobalPositionInt(message);
            break;
            
        case MAVLINK_MSG_ID_HIGH_LATENCY2:
            _handleHighLatency2(message);
            break;
    }
}

void VehicleGPSFactGroup::_handleGPSRawInt(const mavlink_message_t &message)
{
    mavlink_gps_raw_int_t gpsRaw;
    mavlink_msg_gps_raw_int_decode(&message, &gpsRaw);
    
    lat()->setRawValue(static_cast<int32_t>(gpsRaw.lat));
    lon()->setRawValue(static_cast<int32_t>(gpsRaw.lon));
    alt()->setRawValue(static_cast<int32_t>(gpsRaw.alt));
    // alt_ellipsoid field is not available in this MAVLink version
    // altEllipsoid()->setRawValue(static_cast<int32_t>(gpsRaw.alt_ellipsoid));
    hdop()->setRawValue(static_cast<uint16_t>(gpsRaw.eph / 10)); // Convert to dm
    vdop()->setRawValue(static_cast<uint16_t>(gpsRaw.epv / 10)); // Convert to dm
    course()->setRawValue(static_cast<float>(gpsRaw.cog / 100.0f)); // Convert to degrees
    groundSpeed()->setRawValue(static_cast<float>(gpsRaw.vel / 100.0f)); // Convert to m/s
    satellitesVisible()->setRawValue(static_cast<uint8_t>(gpsRaw.satellites_visible));
    fixType()->setRawValue(static_cast<uint8_t>(gpsRaw.fix_type));
    
    _setTelemetryAvailable(true);
}

void VehicleGPSFactGroup::_handleGPS2Raw(const mavlink_message_t &message)
{
    mavlink_gps2_raw_t gps2Raw;
    mavlink_msg_gps2_raw_decode(&message, &gps2Raw);
    
    lat()->setRawValue(static_cast<int32_t>(gps2Raw.lat));
    lon()->setRawValue(static_cast<int32_t>(gps2Raw.lon));
    alt()->setRawValue(static_cast<int32_t>(gps2Raw.alt));
    // alt_ellipsoid field is not available in this MAVLink version
    // altEllipsoid()->setRawValue(static_cast<int32_t>(gps2Raw.alt_ellipsoid));
    hdop()->setRawValue(static_cast<uint16_t>(gps2Raw.eph / 10)); // Convert to dm
    vdop()->setRawValue(static_cast<uint16_t>(gps2Raw.epv / 10)); // Convert to dm
    course()->setRawValue(static_cast<float>(gps2Raw.cog / 100.0f)); // Convert to degrees
    groundSpeed()->setRawValue(static_cast<float>(gps2Raw.vel / 100.0f)); // Convert to m/s
    satellitesVisible()->setRawValue(static_cast<uint8_t>(gps2Raw.satellites_visible));
    fixType()->setRawValue(static_cast<uint8_t>(gps2Raw.fix_type));
    
    _setTelemetryAvailable(true);
}

void VehicleGPSFactGroup::_handleGlobalPositionInt(const mavlink_message_t &message)
{
    mavlink_global_position_int_t globalPos;
    mavlink_msg_global_position_int_decode(&message, &globalPos);
    
    lat()->setRawValue(static_cast<int32_t>(globalPos.lat));
    lon()->setRawValue(static_cast<int32_t>(globalPos.lon));
    alt()->setRawValue(static_cast<int32_t>(globalPos.alt));
    altEllipsoid()->setRawValue(static_cast<int32_t>(globalPos.relative_alt));
    heading()->setRawValue(static_cast<float>(globalPos.hdg / 100.0f)); // Convert to degrees
    
    _setTelemetryAvailable(true);
}

void VehicleGPSFactGroup::_handleHighLatency2(const mavlink_message_t &message)
{
    mavlink_high_latency2_t highLatency2;
    mavlink_msg_high_latency2_decode(&message, &highLatency2);
    
    lat()->setRawValue(static_cast<int32_t>(highLatency2.latitude * 1e7));
    lon()->setRawValue(static_cast<int32_t>(highLatency2.longitude * 1e7));
    alt()->setRawValue(static_cast<int32_t>(highLatency2.altitude * 1000.0f)); // Convert to mm
    groundSpeed()->setRawValue(static_cast<float>(highLatency2.groundspeed));
    heading()->setRawValue(static_cast<float>(highLatency2.heading));
    
    _setTelemetryAvailable(true);
}

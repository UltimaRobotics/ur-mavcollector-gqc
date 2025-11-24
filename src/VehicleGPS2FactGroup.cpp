#include "VehicleGPS2FactGroup.h"
#include "Vehicle.h"
#include <mavlink/v2.0/common/mavlink.h>

VehicleGPS2FactGroup::VehicleGPS2FactGroup(bool ignoreCamelCase)
    : FactGroup(1000, ignoreCamelCase)
{
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

void VehicleGPS2FactGroup::handleMessage(Vehicle *vehicle, const mavlink_message_t &message)
{
    switch (message.msgid) {
        case MAVLINK_MSG_ID_GPS2_RAW:
            _handleGPS2Raw(message);
            break;
    }
}

void VehicleGPS2FactGroup::_handleGPS2Raw(const mavlink_message_t &message)
{
    mavlink_gps2_raw_t gps2Raw;
    mavlink_msg_gps2_raw_decode(&message, &gps2Raw);
    
    lat()->setRawValue(static_cast<int32_t>(gps2Raw.lat));
    lon()->setRawValue(static_cast<int32_t>(gps2Raw.lon));
    alt()->setRawValue(static_cast<int32_t>(gps2Raw.alt));
    // alt_ellipsoid field is not available in this MAVLink version
    // altEllipsoid()->setRawValue(static_cast<int32_t>(gps2Raw.alt_ellipsoid));
    hdop()->setRawValue(static_cast<uint16_t>(gps2Raw.eph / 10));
    vdop()->setRawValue(static_cast<uint16_t>(gps2Raw.epv / 10));
    course()->setRawValue(static_cast<float>(gps2Raw.cog / 100.0f));
    groundSpeed()->setRawValue(static_cast<float>(gps2Raw.vel / 100.0f));
    satellitesVisible()->setRawValue(static_cast<uint8_t>(gps2Raw.satellites_visible));
    fixType()->setRawValue(static_cast<uint8_t>(gps2Raw.fix_type));
    
    _setTelemetryAvailable(true);
}

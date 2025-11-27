#include "VehicleGPSFactGroup.h"
#include "Vehicle.h"

// Include specific MAVLink GPS headers (use v1 to match existing codebase)
#include "../thirdparty/c_library_v2/common/mavlink_msg_gps_raw_int.h"
#include "../thirdparty/c_library_v2/common/mavlink_msg_gps2_raw.h"
#include "../thirdparty/c_library_v2/standard/mavlink_msg_global_position_int.h"
#include "../thirdparty/c_library_v2/common/mavlink_msg_high_latency2.h"
#include "../thirdparty/c_library_v2/common/mavlink_msg_gps_status.h"

#include <iostream>     // For debug output logging (std::cout)
#include <memory>      // For std::shared_ptr used in fact creation
#include <string>      // For string operations in satellite fact names
#include <iomanip>     // For output formatting (std::setw, std::setprecision)
#include <algorithm>   // For potential future algorithms (std::max_element)
#include <cstring>     // For memcpy in packed structure handling

VehicleGPSFactGroup::VehicleGPSFactGroup(bool ignoreCamelCase)
    : FactGroup(1000, ignoreCamelCase) // Update every 1 second
{
    // Add all basic GPS facts
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
    
    // Add GPS status facts for detailed satellite information
    _addFact(std::make_shared<Fact>(0, "gpsStatusSatellitesVisible", FactMetaData::valueTypeUint8));
    _addFact(std::make_shared<Fact>(0, "gpsStatusSatellitesUsed", FactMetaData::valueTypeUint8));
    _addFact(std::make_shared<Fact>(0, "gpsStatusAvgSNR", FactMetaData::valueTypeFloat));
    _addFact(std::make_shared<Fact>(0, "gpsStatusMaxSNR", FactMetaData::valueTypeUint8));
    
    // Initialize GPS status facts to default values
    gpsStatusSatellitesVisible()->setRawValue(static_cast<uint8_t>(0));
    gpsStatusSatellitesUsed()->setRawValue(static_cast<uint8_t>(0));
    gpsStatusAvgSNR()->setRawValue(0.0f);
    gpsStatusMaxSNR()->setRawValue(static_cast<uint8_t>(0));
}

void VehicleGPSFactGroup::handleMessage([[maybe_unused]] Vehicle *vehicle, const mavlink_message_t &message)
{
    switch (message.msgid) {
        case MAVLINK_MSG_ID_GPS_RAW_INT:
            std::cout << "[GPS] Handling GPS_RAW_INT message" << std::endl;
            _handleGPSRawInt(message);
            break;
            
        case MAVLINK_MSG_ID_GPS2_RAW:
            std::cout << "[GPS] Handling GPS2_RAW message" << std::endl;
            _handleGPS2Raw(message);
            break;
            
        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
            std::cout << "[GPS] Handling GLOBAL_POSITION_INT message" << std::endl;
            _handleGlobalPositionInt(message);
            break;
            
        case MAVLINK_MSG_ID_HIGH_LATENCY2:
            std::cout << "[GPS] Handling HIGH_LATENCY2 message" << std::endl;
            _handleHighLatency2(message);
            break;
            
        case MAVLINK_MSG_ID_GPS_STATUS:
            std::cout << "[GPS] Handling GPS_STATUS message - detailed satellite data" << std::endl;
            _handleGPSStatus(message);
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

// GPS status handler for detailed satellite information
void VehicleGPSFactGroup::_handleGPSStatus(const mavlink_message_t &message)
{
    mavlink_gps_status_t gpsStatus;
    mavlink_msg_gps_status_decode(&message, &gpsStatus);
    
    std::cout << "[GPS] === GPS_STATUS Message Decoded ===" << std::endl;
    std::cout << "[GPS] Satellites Visible: " << static_cast<int>(gpsStatus.satellites_visible) << std::endl;
    
    // Count used satellites and calculate SNR statistics
    uint8_t usedCount = 0;
    uint16_t totalSNR = 0;
    uint8_t maxSNR = 0;
    
    for (uint8_t i = 0; i < gpsStatus.satellites_visible && i < 20; i++) {
        if (gpsStatus.satellite_used[i] > 0) {
            usedCount++;
        }
        if (gpsStatus.satellite_snr[i] > 0) {
            totalSNR += gpsStatus.satellite_snr[i];
            if (gpsStatus.satellite_snr[i] > maxSNR) {
                maxSNR = gpsStatus.satellite_snr[i];
            }
        }
        
        std::cout << "[GPS] Sat " << std::setw(2) << (i+1) 
                  << " PRN: " << std::setw(3) << static_cast<int>(gpsStatus.satellite_prn[i])
                  << " Used: " << (gpsStatus.satellite_used[i] ? "Yes" : "No ")
                  << " Elev: " << std::setw(3) << static_cast<int>(gpsStatus.satellite_elevation[i]) << "°"
                  << " Azim: " << std::setw(3) << static_cast<int>(gpsStatus.satellite_azimuth[i]) << "°"
                  << " SNR: " << std::setw(3) << static_cast<int>(gpsStatus.satellite_snr[i]) << "dB"
                  << std::endl;
    }
    
    // Calculate average SNR
    float avgSNR = 0.0f;
    if (gpsStatus.satellites_visible > 0) {
        avgSNR = static_cast<float>(totalSNR) / static_cast<float>(gpsStatus.satellites_visible);
    }
    
    std::cout << "[GPS] Used Satellites: " << static_cast<int>(usedCount) << std::endl;
    std::cout << "[GPS] Average SNR: " << std::fixed << std::setprecision(1) << avgSNR << "dB" << std::endl;
    std::cout << "[GPS] Max SNR: " << static_cast<int>(maxSNR) << "dB" << std::endl;
    std::cout << "[GPS] =====================================" << std::endl;
    
    // Update GPS status facts
    _updateGPSStatusFacts(gpsStatus);
    
    // Update individual satellite facts
    _updateSatelliteFacts(gpsStatus);
    
    _setTelemetryAvailable(true);
}

// Update GPS status facts with satellite statistics
void VehicleGPSFactGroup::_updateGPSStatusFacts(const __mavlink_gps_status_t& gpsStatus)
{
    // Create a typed copy for easier access
    mavlink_gps_status_t gpsStatusTyped;
    std::memcpy(&gpsStatusTyped, &gpsStatus, sizeof(gpsStatusTyped));
    
    // Update basic GPS status facts
    gpsStatusSatellitesVisible()->setRawValue(gpsStatusTyped.satellites_visible);
    
    // Count used satellites
    uint8_t usedCount = 0;
    for (uint8_t i = 0; i < gpsStatusTyped.satellites_visible && i < 20; i++) {
        if (gpsStatusTyped.satellite_used[i] > 0) {
            usedCount++;
        }
    }
    gpsStatusSatellitesUsed()->setRawValue(usedCount);
    
    // Calculate SNR statistics
    uint16_t totalSNR = 0;
    uint8_t maxSNR = 0;
    uint8_t validSNRCount = 0;
    
    for (uint8_t i = 0; i < gpsStatusTyped.satellites_visible && i < 20; i++) {
        if (gpsStatusTyped.satellite_snr[i] > 0) {
            totalSNR += gpsStatusTyped.satellite_snr[i];
            validSNRCount++;
            if (gpsStatusTyped.satellite_snr[i] > maxSNR) {
                maxSNR = gpsStatusTyped.satellite_snr[i];
            }
        }
    }
    
    // Calculate average SNR
    float avgSNR = 0.0f;
    if (validSNRCount > 0) {
        avgSNR = static_cast<float>(totalSNR) / static_cast<float>(validSNRCount);
    }
    
    gpsStatusAvgSNR()->setRawValue(avgSNR);
    gpsStatusMaxSNR()->setRawValue(maxSNR);
}

// Add satellite facts for the specified number of satellites
void VehicleGPSFactGroup::_addSatelliteFacts(uint8_t satelliteCount)
{
    for (uint8_t i = 0; i < satelliteCount && i < 20; i++) {
        std::string prnFactName = "satellite" + std::to_string(i) + "PRN";
        std::string usedFactName = "satellite" + std::to_string(i) + "Used";
        std::string elevFactName = "satellite" + std::to_string(i) + "Elevation";
        std::string azimFactName = "satellite" + std::to_string(i) + "Azimuth";
        std::string snrFactName = "satellite" + std::to_string(i) + "SNR";
        
        // Only add if they don't already exist
        if (!getFact(prnFactName)) {
            _addFact(std::make_shared<Fact>(0, prnFactName, FactMetaData::valueTypeUint8));
        }
        if (!getFact(usedFactName)) {
            _addFact(std::make_shared<Fact>(0, usedFactName, FactMetaData::valueTypeUint8));
        }
        if (!getFact(elevFactName)) {
            _addFact(std::make_shared<Fact>(0, elevFactName, FactMetaData::valueTypeUint8));
        }
        if (!getFact(azimFactName)) {
            _addFact(std::make_shared<Fact>(0, azimFactName, FactMetaData::valueTypeUint8));
        }
        if (!getFact(snrFactName)) {
            _addFact(std::make_shared<Fact>(0, snrFactName, FactMetaData::valueTypeUint8));
        }
    }
    
    // Update max satellite count
    if (satelliteCount > _maxSatellites) {
        _maxSatellites = satelliteCount;
    }
}

// Update individual satellite facts
void VehicleGPSFactGroup::_updateSatelliteFacts(const __mavlink_gps_status_t& gpsStatus)
{
    // Create a typed copy for easier access
    mavlink_gps_status_t gpsStatusTyped;
    std::memcpy(&gpsStatusTyped, &gpsStatus, sizeof(gpsStatusTyped));
    
    // Add facts for all visible satellites
    _addSatelliteFacts(gpsStatusTyped.satellites_visible);
    
    // Update individual satellite data
    for (uint8_t i = 0; i < gpsStatusTyped.satellites_visible && i < 20; i++) {
        std::string prnFactName = "satellite" + std::to_string(i) + "PRN";
        std::string usedFactName = "satellite" + std::to_string(i) + "Used";
        std::string elevFactName = "satellite" + std::to_string(i) + "Elevation";
        std::string azimFactName = "satellite" + std::to_string(i) + "Azimuth";
        std::string snrFactName = "satellite" + std::to_string(i) + "SNR";
        
        auto prnFact = getFact(prnFactName);
        auto usedFact = getFact(usedFactName);
        auto elevFact = getFact(elevFactName);
        auto azimFact = getFact(azimFactName);
        auto snrFact = getFact(snrFactName);
        
        if (prnFact) prnFact->setRawValue(gpsStatusTyped.satellite_prn[i]);
        if (usedFact) usedFact->setRawValue(gpsStatusTyped.satellite_used[i]);
        if (elevFact) elevFact->setRawValue(gpsStatusTyped.satellite_elevation[i]);
        if (azimFact) azimFact->setRawValue(gpsStatusTyped.satellite_azimuth[i]);
        if (snrFact) snrFact->setRawValue(gpsStatusTyped.satellite_snr[i]);
    }
}

// Public methods to access individual satellite data
std::shared_ptr<Fact> VehicleGPSFactGroup::satellitePRN(uint8_t index)
{
    if (index >= 20) return nullptr;
    std::string factName = "satellite" + std::to_string(index) + "PRN";
    return getFact(factName);
}

std::shared_ptr<Fact> VehicleGPSFactGroup::satelliteUsed(uint8_t index)
{
    if (index >= 20) return nullptr;
    std::string factName = "satellite" + std::to_string(index) + "Used";
    return getFact(factName);
}

std::shared_ptr<Fact> VehicleGPSFactGroup::satelliteElevation(uint8_t index)
{
    if (index >= 20) return nullptr;
    std::string factName = "satellite" + std::to_string(index) + "Elevation";
    return getFact(factName);
}

std::shared_ptr<Fact> VehicleGPSFactGroup::satelliteAzimuth(uint8_t index)
{
    if (index >= 20) return nullptr;
    std::string factName = "satellite" + std::to_string(index) + "Azimuth";
    return getFact(factName);
}

std::shared_ptr<Fact> VehicleGPSFactGroup::satelliteSNR(uint8_t index)
{
    if (index >= 20) return nullptr;
    std::string factName = "satellite" + std::to_string(index) + "SNR";
    return getFact(factName);
}

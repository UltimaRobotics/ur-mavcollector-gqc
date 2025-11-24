#pragma once

#include "FactGroup.h"

/// GPS FactGroup containing GPS telemetry data.
/// This is a Qt-free port of QGroundControl's VehicleGPSFactGroup.
class VehicleGPSFactGroup : public FactGroup
{
public:
    explicit VehicleGPSFactGroup(bool ignoreCamelCase = false);
    virtual ~VehicleGPSFactGroup() = default;

    // Fact accessors
    std::shared_ptr<Fact> lat() { return getFact("lat"); }
    std::shared_ptr<Fact> lon() { return getFact("lon"); }
    std::shared_ptr<Fact> alt() { return getFact("alt"); }
    std::shared_ptr<Fact> altEllipsoid() { return getFact("altEllipsoid"); }
    std::shared_ptr<Fact> hdop() { return getFact("hdop"); }
    std::shared_ptr<Fact> vdop() { return getFact("vdop"); }
    std::shared_ptr<Fact> course() { return getFact("course"); }
    std::shared_ptr<Fact> groundSpeed() { return getFact("groundSpeed"); }
    std::shared_ptr<Fact> count() { return getFact("count"); }
    std::shared_ptr<Fact> lock() { return getFact("lock"); }
    std::shared_ptr<Fact> satellitesVisible() { return getFact("satellitesVisible"); }
    std::shared_ptr<Fact> utcDate() { return getFact("utcDate"); }
    std::shared_ptr<Fact> utcTime() { return getFact("utcTime"); }
    std::shared_ptr<Fact> timeUtc() { return getFact("timeUtc"); }
    std::shared_ptr<Fact> fixType() { return getFact("fixType"); }
    std::shared_ptr<Fact> eph() { return getFact("eph"); }
    std::shared_ptr<Fact> epv() { return getFact("epv"); }
    std::shared_ptr<Fact> heading() { return getFact("heading"); }
    std::shared_ptr<Fact> speedAccuracy() { return getFact("speedAccuracy"); }
    std::shared_ptr<Fact> horizAccuracy() { return getFact("horizAccuracy"); }
    std::shared_ptr<Fact> vertAccuracy() { return getFact("vertAccuracy"); }
    std::shared_ptr<Fact> yaw() { return getFact("yaw"); }
    std::shared_ptr<Fact> yawAccuracy() { return getFact("yawAccuracy"); }

    void handleMessage(Vehicle *vehicle, const mavlink_message_t &message) override;

protected:
    void _handleGPSRawInt(const mavlink_message_t &message);
    void _handleGPS2Raw(const mavlink_message_t &message);
    void _handleGlobalPositionInt(const mavlink_message_t &message);
    void _handleHighLatency2(const mavlink_message_t &message);
};

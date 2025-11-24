#pragma once

#include "FactGroup.h"

/// Battery FactGroup containing battery telemetry data.
/// This is a Qt-free port of QGroundControl's BatteryFactGroup.
class VehicleBatteryFactGroup : public FactGroup
{
public:
    explicit VehicleBatteryFactGroup(bool ignoreCamelCase = false);
    virtual ~VehicleBatteryFactGroup() = default;

    // Fact accessors
    std::shared_ptr<Fact> voltage() { return getFact("voltage"); }
    std::shared_ptr<Fact> current() { return getFact("current"); }
    std::shared_ptr<Fact> consumed() { return getFact("consumed"); }
    std::shared_ptr<Fact> remaining() { return getFact("remaining"); }
    std::shared_ptr<Fact> percent() { return getFact("percent"); }
    std::shared_ptr<Fact> temperature() { return getFact("temperature"); }
    std::shared_ptr<Fact> id() { return getFact("id"); }
    std::shared_ptr<Fact> function() { return getFact("function"); }
    std::shared_ptr<Fact> type() { return getFact("type"); }
    std::shared_ptr<Fact> timeRemaining() { return getFact("timeRemaining"); }
    std::shared_ptr<Fact> chargeState() { return getFact("chargeState"); }

    void handleMessage(Vehicle *vehicle, const mavlink_message_t &message) override;

protected:
    void _handleBatteryStatus(const mavlink_message_t &message);
    void _handleSysStatus(const mavlink_message_t &message);
};

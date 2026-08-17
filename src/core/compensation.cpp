#include "compensation.hpp"

namespace gutcpp {

FlightTimeCompensator::FlightTimeCompensator(const CompensationConfig& config)
    : config_(config) {}

double FlightTimeCompensator::totalDelay() const {
    const double bulletFlightTime = (config_.bulletSpeed > 0.0)
        ? (config_.targetDistance / config_.bulletSpeed)
        : 0.0;
    return bulletFlightTime + config_.commLatencySec + config_.gimbalDelaySec + config_.extraDelaySec;
}

double FlightTimeCompensator::computeAngleOffset(double angularVelocity) const {
    return angularVelocity * totalDelay();
}

void FlightTimeCompensator::updateConfig(const CompensationConfig& config) {
    config_ = config;
}

} // namespace gutcpp

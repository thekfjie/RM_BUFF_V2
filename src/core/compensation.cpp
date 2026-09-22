#include "compensation.hpp"
#include <cmath>
#include <stdexcept>

namespace gutcpp {

FlightTimeCompensator::FlightTimeCompensator(const CompensationConfig& config)
    : config_(config) {}

double FlightTimeCompensator::totalDelay() const {
    if (!std::isfinite(config_.bulletSpeed) || config_.bulletSpeed <= 0.0 ||
        !std::isfinite(config_.targetDistance) || config_.targetDistance <= 0.0 ||
        !std::isfinite(config_.commLatencySec) || config_.commLatencySec < 0.0 ||
        !std::isfinite(config_.gimbalDelaySec) || config_.gimbalDelaySec < 0.0 ||
        !std::isfinite(config_.extraDelaySec) || config_.extraDelaySec < 0.0)
        throw std::invalid_argument("BUFF compensation requires finite positive speed/distance and nonnegative delays");
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

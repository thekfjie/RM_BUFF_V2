#pragma once

namespace gutcpp {

struct CompensationConfig {
    double bulletSpeed = 15.0;       // m/s
    double targetDistance = 7.0;     // meters
    double commLatencySec = 0.01;    // communication latency
    double gimbalDelaySec = 0.05;    // gimbal response delay
    double extraDelaySec = 0.0;      // user-defined additional delay
};

class FlightTimeCompensator {
public:
    explicit FlightTimeCompensator(const CompensationConfig& config = {});

    double computeAngleOffset(double angularVelocity) const;
    double totalDelay() const;
    void updateConfig(const CompensationConfig& config);

private:
    CompensationConfig config_;
};

} // namespace gutcpp

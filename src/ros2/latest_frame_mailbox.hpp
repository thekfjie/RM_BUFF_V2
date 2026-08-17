#pragma once

#include <condition_variable>
#include <mutex>
#include <optional>
#include <utility>

#include "buff_ros_utils.hpp"

namespace gutcpp {

class LatestFrameMailbox {
public:
    void put(FramePacket packet) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            latestFrame_ = std::move(packet);
        }
        cv_.notify_one();
    }

    std::optional<FramePacket> waitAndTake() {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait(lock, [this] {
            return stopped_ || latestFrame_.has_value();
        });
        if (stopped_) {
            return std::nullopt;
        }

        std::optional<FramePacket> packet = std::move(latestFrame_);
        latestFrame_.reset();
        return packet;
    }

    void stop() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stopped_ = true;
            latestFrame_.reset();
        }
        cv_.notify_all();
    }

private:
    std::mutex mutex_;
    std::condition_variable cv_;
    std::optional<FramePacket> latestFrame_;
    bool stopped_ = false;
};

} // namespace gutcpp

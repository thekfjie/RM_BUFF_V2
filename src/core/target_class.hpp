#pragma once

#include <stdexcept>
#include <string>

namespace gutcpp {

// Local five-keypoint model labels. These are NOT the HW nine-keypoint IDs.
enum class TargetClass : int { RedTarget = 0, RedHit = 1, BlueTarget = 2, BlueHit = 3 };

inline bool IsUnhitTargetClass(int id) {
    return id == static_cast<int>(TargetClass::RedTarget) ||
           id == static_cast<int>(TargetClass::BlueTarget);
}

inline int TargetClassForColor(const std::string& targetColor) {
    if (targetColor == "red") return static_cast<int>(TargetClass::RedTarget);
    if (targetColor == "blue") return static_cast<int>(TargetClass::BlueTarget);
    throw std::invalid_argument("BUFF target color must be red or blue");
}

} // namespace gutcpp

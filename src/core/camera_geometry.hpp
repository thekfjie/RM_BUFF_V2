#pragma once

#include <optional>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "types.hpp"

namespace gutcpp {

struct CameraModel {
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    int imageWidth = 0;
    int imageHeight = 0;
    std::string frameId;
};

struct RayProjection {
    bool valid = false;
    double yaw = 0.0;    // rad, positive to camera right in optical frame
    double pitch = 0.0;  // rad, positive upward in optical frame
    cv::Point3d ray{0.0, 0.0, 1.0};
    cv::Point3d point{0.0, 0.0, 0.0};
};

struct PnpResult {
    bool valid = false;
    cv::Mat rvec;
    cv::Mat tvec;
};

bool IsCameraModelUsable(const CameraModel& camera);

RayProjection ProjectPixelToRay(const CameraModel& camera,
                                const cv::Point2d& pixel,
                                double targetDistance);

RayProjection ProjectCameraPointToAngles(const cv::Point3d& point);

std::optional<PnpResult> SolveBuffPnp(const CameraModel& camera,
                                      const Keypoints& keypoints,
                                      const std::vector<cv::Point3f>& objectPoints);

} // namespace gutcpp

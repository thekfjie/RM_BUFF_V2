#include "camera_geometry.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cmath>
#include <iterator>

#include <opencv2/calib3d.hpp>

namespace gutcpp {

namespace {

constexpr std::array<int, 4> kBladeKeypointIndices = {0, 1, 3, 4};

bool HasFinitePositiveDepth(const cv::Point3d& point) {
    return std::isfinite(point.x) && std::isfinite(point.y) &&
           std::isfinite(point.z) && point.z > 1e-6;
}

} // namespace

bool IsCameraModelUsable(const CameraModel& camera) {
    return !camera.cameraMatrix.empty() &&
           camera.cameraMatrix.rows == 3 &&
           camera.cameraMatrix.cols == 3 &&
           camera.cameraMatrix.type() == CV_64F &&
           std::isfinite(camera.cameraMatrix.at<double>(0, 0)) &&
           std::isfinite(camera.cameraMatrix.at<double>(1, 1)) &&
           camera.cameraMatrix.at<double>(0, 0) > 0.0 &&
           camera.cameraMatrix.at<double>(1, 1) > 0.0;
}

RayProjection ProjectCameraPointToAngles(const cv::Point3d& point) {
    RayProjection projection;
    if (!HasFinitePositiveDepth(point)) {
        return projection;
    }

    const double horizontalRange = std::hypot(point.x, point.z);
    const double range = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
    projection.valid = true;
    projection.yaw = std::atan2(point.x, point.z);
    projection.pitch = std::atan2(-point.y, horizontalRange);
    projection.ray = point * (1.0 / range);
    projection.point = point;
    return projection;
}

RayProjection ProjectPixelToRay(const CameraModel& camera,
                                const cv::Point2d& pixel,
                                double targetDistance) {
    RayProjection projection;
    if (!IsCameraModelUsable(camera) || !std::isfinite(pixel.x) || !std::isfinite(pixel.y)) {
        return projection;
    }

    std::vector<cv::Point2f> sourcePoints{
        cv::Point2f(static_cast<float>(pixel.x), static_cast<float>(pixel.y))
    };
    std::vector<cv::Point2f> normalizedPoints;
    cv::undistortPoints(sourcePoints, normalizedPoints, camera.cameraMatrix, camera.distCoeffs);
    if (normalizedPoints.empty()) {
        return projection;
    }

    const cv::Point2f normalized = normalizedPoints.front();
    const cv::Point3d ray(static_cast<double>(normalized.x),
                          static_cast<double>(normalized.y),
                          1.0);
    const double rayNorm = std::sqrt(ray.x * ray.x + ray.y * ray.y + ray.z * ray.z);
    const cv::Point3d cameraPoint = ray * (targetDistance / rayNorm);
    return ProjectCameraPointToAngles(cameraPoint);
}

std::optional<PnpResult> SolveBuffPnp(const CameraModel& camera,
                                      const Keypoints& keypoints,
                                      const std::vector<cv::Point3f>& objectPoints) {
    if (!IsCameraModelUsable(camera) || !keypoints.valid || objectPoints.size() < kBladeKeypointIndices.size()) {
        return std::nullopt;
    }

    std::vector<cv::Point2f> imagePoints;
    imagePoints.reserve(kBladeKeypointIndices.size());
    for (const int keypointIndex : kBladeKeypointIndices) {
        const cv::Point2f point = keypoints.points[static_cast<std::size_t>(keypointIndex)];
        if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
            return std::nullopt;
        }
        imagePoints.push_back(point);
    }

    std::vector<cv::Point3f> selectedObjectPoints;
    selectedObjectPoints.reserve(kBladeKeypointIndices.size());
    std::copy_n(objectPoints.begin(),
                static_cast<std::ptrdiff_t>(kBladeKeypointIndices.size()),
                std::back_inserter(selectedObjectPoints));

    PnpResult result;
    const int method = selectedObjectPoints.size() == 4 ? cv::SOLVEPNP_IPPE : cv::SOLVEPNP_ITERATIVE;
    if (!cv::solvePnP(selectedObjectPoints,
                      imagePoints,
                      camera.cameraMatrix,
                      camera.distCoeffs,
                      result.rvec,
                      result.tvec,
                      false,
                      method)) {
        return std::nullopt;
    }

    result.valid = result.tvec.rows == 3 && result.tvec.cols == 1;
    return result.valid ? std::optional<PnpResult>(result) : std::nullopt;
}

} // namespace gutcpp

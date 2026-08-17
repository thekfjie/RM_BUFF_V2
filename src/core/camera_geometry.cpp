#include "camera_geometry.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cmath>
#include <iterator>
#include <limits>
#include <utility>

#include <opencv2/calib3d.hpp>

namespace gutcpp {

namespace {

constexpr std::array<int, 4> kBladeKeypointIndices = {0, 1, 3, 4};

bool HasFinitePositiveDepth(const cv::Point3d& point) {
    return std::isfinite(point.x) && std::isfinite(point.y) &&
           std::isfinite(point.z) && point.z > 1e-6;
}

bool NormalizePoseVector(const cv::Mat& input, cv::Mat& output) {
    if (input.empty() || input.total() != 3) {
        return false;
    }
    cv::Mat vector = input.reshape(1, 3);
    vector.convertTo(output, CV_64F);
    for (int row = 0; row < 3; ++row) {
        if (!std::isfinite(output.at<double>(row, 0))) {
            return false;
        }
    }
    return true;
}

double ReprojectionError(const std::vector<cv::Point3f>& objectPoints,
                         const std::vector<cv::Point2f>& imagePoints,
                         const cv::Mat& rvec,
                         const cv::Mat& tvec,
                         const CameraModel& camera) {
    std::vector<cv::Point2f> projected;
    cv::projectPoints(objectPoints,
                      rvec,
                      tvec,
                      camera.cameraMatrix,
                      camera.distCoeffs,
                      projected);
    if (projected.size() != imagePoints.size() || projected.empty()) {
        return std::numeric_limits<double>::infinity();
    }

    double squaredError = 0.0;
    for (std::size_t index = 0; index < projected.size(); ++index) {
        const cv::Point2f residual = projected[index] - imagePoints[index];
        squaredError += static_cast<double>(residual.dot(residual));
    }
    return std::sqrt(squaredError / static_cast<double>(projected.size()));
}

double RotationDistance(const cv::Mat& lhsRvec, const cv::Mat& rhsRvec) {
    cv::Mat lhsRotation;
    cv::Mat rhsRotation;
    cv::Rodrigues(lhsRvec, lhsRotation);
    cv::Rodrigues(rhsRvec, rhsRotation);
    const cv::Mat relative = lhsRotation.t() * rhsRotation;
    const double trace = relative.at<double>(0, 0) +
                         relative.at<double>(1, 1) +
                         relative.at<double>(2, 2);
    return std::acos(std::clamp((trace - 1.0) * 0.5, -1.0, 1.0));
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

std::optional<PnpResult> BuffPnpSolver::solve(
    const CameraModel& camera,
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

    std::vector<cv::Mat> candidateRvecs;
    std::vector<cv::Mat> candidateTvecs;
    const int solutionCount = cv::solvePnPGeneric(selectedObjectPoints,
                                                  imagePoints,
                                                  camera.cameraMatrix,
                                                  camera.distCoeffs,
                                                  candidateRvecs,
                                                  candidateTvecs,
                                                  false,
                                                  cv::SOLVEPNP_IPPE);
    const int candidateCount = std::min(
        solutionCount,
        static_cast<int>(std::min(candidateRvecs.size(), candidateTvecs.size())));
    if (candidateCount <= 0) {
        return std::nullopt;
    }

    int selectedIndex = -1;
    double selectedScore = std::numeric_limits<double>::infinity();
    double selectedReprojectionError = std::numeric_limits<double>::infinity();
    cv::Mat selectedRvec;
    cv::Mat selectedTvec;

    for (int index = 0; index < candidateCount; ++index) {
        cv::Mat rvec;
        cv::Mat tvec;
        if (!NormalizePoseVector(candidateRvecs[static_cast<std::size_t>(index)], rvec) ||
            !NormalizePoseVector(candidateTvecs[static_cast<std::size_t>(index)], tvec) ||
            tvec.at<double>(2, 0) <= 1e-6) {
            continue;
        }

        const double reprojectionError = ReprojectionError(selectedObjectPoints,
                                                           imagePoints,
                                                           rvec,
                                                           tvec,
                                                           camera);
        if (!std::isfinite(reprojectionError)) {
            continue;
        }

        double score = reprojectionError;
        if (hasPreviousSolution_) {
            const double previousRange = std::max(0.1, cv::norm(previousTvec_));
            const double translationDistance = cv::norm(tvec - previousTvec_) / previousRange;
            const double rotationDistance = RotationDistance(previousRvec_, rvec);
            score = 2.0 * translationDistance + rotationDistance + 0.02 * reprojectionError;
        }

        if (score < selectedScore) {
            selectedIndex = index;
            selectedScore = score;
            selectedReprojectionError = reprojectionError;
            selectedRvec = rvec;
            selectedTvec = tvec;
        }
    }

    if (selectedIndex < 0) {
        return std::nullopt;
    }

    previousRvec_ = selectedRvec.clone();
    previousTvec_ = selectedTvec.clone();
    hasPreviousSolution_ = true;

    PnpResult result;
    result.valid = true;
    result.rvec = std::move(selectedRvec);
    result.tvec = std::move(selectedTvec);
    result.reprojectionError = selectedReprojectionError;
    result.candidateCount = candidateCount;
    result.selectedIndex = selectedIndex;
    return result;
}

void BuffPnpSolver::reset() {
    hasPreviousSolution_ = false;
    previousRvec_.release();
    previousTvec_.release();
}

std::optional<PnpResult> SolveBuffPnp(const CameraModel& camera,
                                      const Keypoints& keypoints,
                                      const std::vector<cv::Point3f>& objectPoints) {
    BuffPnpSolver solver;
    return solver.solve(camera, keypoints, objectPoints);
}

} // namespace gutcpp

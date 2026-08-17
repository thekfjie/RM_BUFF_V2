#include "core/camera_geometry.hpp"

#include <cmath>
#include <iostream>
#include <vector>

#include <opencv2/calib3d.hpp>

namespace {

bool Near(double lhs, double rhs, double tolerance) {
    return std::fabs(lhs - rhs) <= tolerance;
}

gutcpp::CameraModel MakeTestCamera() {
    gutcpp::CameraModel camera;
    camera.cameraMatrix = (cv::Mat_<double>(3, 3) <<
        1000.0, 0.0, 640.0,
        0.0, 1000.0, 360.0,
        0.0, 0.0, 1.0);
    camera.distCoeffs = cv::Mat::zeros(1, 5, CV_64F);
    camera.imageWidth = 1280;
    camera.imageHeight = 720;
    camera.frameId = "camera_optical_frame";
    return camera;
}

int Fail(const char* message) {
    std::cerr << message << std::endl;
    return 1;
}

} // namespace

int main() {
    const gutcpp::CameraModel camera = MakeTestCamera();
    const gutcpp::RayProjection center =
        gutcpp::ProjectPixelToRay(camera, cv::Point2d(640.0, 360.0), 7.0);
    if (!center.valid || !Near(center.yaw, 0.0, 1e-9) || !Near(center.pitch, 0.0, 1e-9)) {
        return Fail("center pixel should project to zero yaw/pitch");
    }

    const gutcpp::RayProjection right =
        gutcpp::ProjectPixelToRay(camera, cv::Point2d(1640.0, 360.0), 7.0);
    if (!right.valid || !Near(right.yaw, CV_PI / 4.0, 1e-6) || !Near(right.pitch, 0.0, 1e-9)) {
        return Fail("one focal length right should project to 45 degree yaw");
    }

    const gutcpp::RayProjection up =
        gutcpp::ProjectPixelToRay(camera, cv::Point2d(640.0, -640.0), 7.0);
    if (!up.valid || !Near(up.pitch, CV_PI / 4.0, 1e-6)) {
        return Fail("one focal length up should project to 45 degree positive pitch");
    }

    const std::vector<cv::Point3f> objectPoints{
        {-0.114f, -0.114f, 0.0f},
        { 0.114f, -0.114f, 0.0f},
        { 0.114f,  0.114f, 0.0f},
        {-0.114f,  0.114f, 0.0f},
    };
    const cv::Mat expectedRvec = (cv::Mat_<double>(3, 1) << 0.08, -0.12, 0.03);
    const cv::Mat expectedTvec = (cv::Mat_<double>(3, 1) << 0.10, -0.05, 3.0);
    std::vector<cv::Point2f> projected;
    cv::projectPoints(objectPoints,
                      expectedRvec,
                      expectedTvec,
                      camera.cameraMatrix,
                      camera.distCoeffs,
                      projected);

    gutcpp::Keypoints keypoints;
    keypoints.valid = true;
    keypoints.points[0] = projected[0];
    keypoints.points[1] = projected[1];
    keypoints.points[3] = projected[2];
    keypoints.points[4] = projected[3];

    gutcpp::BuffPnpSolver solver;
    const std::optional<gutcpp::PnpResult> pose = solver.solve(camera, keypoints, objectPoints);
    if (!pose.has_value() || !pose->valid || pose->candidateCount < 1 ||
        pose->selectedIndex < 0 || pose->tvec.at<double>(2, 0) <= 0.0 ||
        pose->reprojectionError > 1e-3) {
        return Fail("IPPE solver should return a finite positive-depth candidate");
    }
    if (cv::norm(pose->tvec - expectedTvec) > 1e-2) {
        return Fail("IPPE solver translation should match the synthetic pose");
    }

    const cv::Mat nextTvec = (cv::Mat_<double>(3, 1) << 0.11, -0.045, 3.01);
    cv::projectPoints(objectPoints,
                      expectedRvec,
                      nextTvec,
                      camera.cameraMatrix,
                      camera.distCoeffs,
                      projected);
    keypoints.points[0] = projected[0];
    keypoints.points[1] = projected[1];
    keypoints.points[3] = projected[2];
    keypoints.points[4] = projected[3];
    const std::optional<gutcpp::PnpResult> nextPose = solver.solve(camera, keypoints, objectPoints);
    if (!nextPose.has_value() || cv::norm(nextPose->tvec - nextTvec) > 1e-2) {
        return Fail("IPPE solver should preserve the continuous synthetic branch");
    }

    return 0;
}

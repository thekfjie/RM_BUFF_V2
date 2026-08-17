#include "core/camera_geometry.hpp"

#include <cmath>
#include <iostream>
#include <vector>

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

    return 0;
}

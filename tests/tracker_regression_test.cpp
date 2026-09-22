#include "core/buff_tracker.hpp"
#include "core/target_class.hpp"

#include <cmath>
#include <iostream>
#include <opencv2/imgproc.hpp>

int main() {
    if (gutcpp::TargetClassForColor("red") != 0 || gutcpp::TargetClassForColor("blue") != 2 ||
        gutcpp::IsUnhitTargetClass(1) || gutcpp::IsUnhitTargetClass(3)) return 1;
    gutcpp::Parameter parameter;
    parameter.hsv.lowerLimit = cv::Scalar(0, 100, 100);
    parameter.hsv.upperLimit = cv::Scalar(15, 255, 255);
    parameter.insideRate = 0.6;
    parameter.outsideRate = 1.5;
    parameter.kernel = 0;
    parameter.maybeTarget.area = 0.5;
    parameter.maybeTarget.width = 0.5;
    parameter.maybeTarget.height = 0.5;
    // Two visible candidates: initialization must associate the supplied seed,
    // not require the complete image to contain only one blade.
    cv::Mat frame = cv::Mat::zeros(600, 600, CV_8UC3);
    cv::rectangle(frame, cv::Rect(295, 295, 10, 10), cv::Scalar(0, 0, 255), cv::FILLED);
    cv::rectangle(frame, cv::Rect(395, 290, 50, 20), cv::Scalar(0, 0, 255), cv::FILLED);
    cv::rectangle(frame, cv::Rect(327, 389, 20, 50), cv::Scalar(0, 0, 255), cv::FILLED);
    gutcpp::F_BuffTracker tracker({395, 290, 445, 310}, {295, 295, 305, 305}, parameter, false);
    if (!tracker.update(frame, true) || cv::norm(tracker.fanBladeBox().center2f() - cv::Point2f(420, 300)) > 3.0) {
        std::cerr << "multiple-blade seed association failed: " << tracker.lastFailureReason() << '\n';
        return 1;
    }
    if (!tracker.update(frame, true) || cv::norm(tracker.fanBladeBox().center2f() - cv::Point2f(420, 300)) > 3.0) {
        std::cerr << "additional contours must not be mistaken for a hit/target switch\n";
        return 1;
    }
    cv::Mat empty = cv::Mat::zeros(600, 600, CV_8UC3);
    if (tracker.update(empty, true)) {
        std::cerr << "a missing target must not be reported as observed\n";
        return 1;
    }
    return 0;
}

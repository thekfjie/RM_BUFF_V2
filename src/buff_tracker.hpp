#pragma once

#include <optional>
#include <string>
#include <vector>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "parameter.hpp"

namespace gutcpp {

enum class IoUType {
    IoU,
    GIoU,
    DIoU,
    CIoU,
};

class BBox {
public:
    BBox() = default;
    BBox(double xmin, double ymin, double xmax, double ymax, int bboxId = -1);

    [[nodiscard]] double intersectionArea(const BBox& other) const;
    [[nodiscard]] double unionArea(const BBox& other) const;
    [[nodiscard]] double iou(const BBox& other) const;
    [[nodiscard]] BBox boundOf(const BBox& other) const;
    [[nodiscard]] double centerDistance(const BBox& other) const;
    [[nodiscard]] double boundDiagonalDistance(const BBox& other) const;
    [[nodiscard]] cv::Point2f center2f() const;
    [[nodiscard]] cv::Point center2i() const;
    [[nodiscard]] double area() const;
    [[nodiscard]] double width() const;
    [[nodiscard]] double height() const;
    [[nodiscard]] BBox createNewBBoxByCenter(const cv::Point2f& center) const;
    [[nodiscard]] cv::Point p1i() const;
    [[nodiscard]] cv::Point p2i() const;

    double xmin = 0.0;
    double ymin = 0.0;
    double xmax = 0.0;
    double ymax = 0.0;
    int id = -1;
};

struct RotationRectangle {
    RotationRectangle() = default;
    RotationRectangle(const std::vector<cv::Point2f>& points, const cv::Point2f& rBoxCenter);

    [[nodiscard]] cv::Point2f center2f() const;
    [[nodiscard]] cv::Point center2i() const;
    [[nodiscard]] double width() const;
    [[nodiscard]] double height() const;
    [[nodiscard]] double area() const;

    cv::Point2f p1;
    cv::Point2f p2;
    cv::Point2f p3;
    cv::Point2f p4;
    std::vector<cv::Point2f> points;
    cv::Point2f top;
    double disTop = 0.0;
    cv::Point2f btm;
    double disBtm = 0.0;

private:
    static cv::Point2f lineCenter(const cv::Point2f& p1, const cv::Point2f& p2);
};

struct FanBlade {
    BBox bbox;
    RotationRectangle rtnRect;
};

struct TargetStruct {
    BBox box;
    double iou = 0.0;
};

double IoU(const BBox& a, const BBox& b);
double GIoU(const BBox& a, const BBox& b);
double DIoU(const BBox& a, const BBox& b);
double CIoU(const BBox& a, const BBox& b);
std::vector<TargetStruct> CompareByIoU(const BBox& lastBox, const std::vector<BBox>& boxes, IoUType type);

class F_BuffTracker {
public:
    F_BuffTracker(BBox fanBladeBox, BBox rBox, Parameter parameter, bool isImshow = true);

    bool update(cv::Mat& frame, bool isOpenMaybeTarget);

    [[nodiscard]] const BBox& fanBladeBox() const;
    [[nodiscard]] const BBox& rBox() const;
    [[nodiscard]] double radius() const;
    [[nodiscard]] const std::string& lastFailureReason() const;

private:
    static BBox pointsToBBox(const std::vector<cv::Point2f>& points);
    [[nodiscard]] cv::Mat getMaskByHSVThreshold(const cv::Mat& frame) const;
    std::optional<std::vector<FanBlade>> getFanBlade(cv::Mat& mask);
    [[nodiscard]] bool mayBeTarget(double width, double height, bool flag) const;
    [[nodiscard]] std::vector<BBox> getAlternateBoxes(const cv::Mat& mask, bool flag) const;
    static RotationRectangle makeDummyRotationRect(const cv::Point2f& rBoxCenter);
    static void safeImshow(const std::string& windowName, const cv::Mat& image);

    Parameter parameter_;
    BBox fanBladeBox_;
    BBox rBox_;
    std::vector<FanBlade> fanBladeList_;
    double radius_ = 0.0;
    std::vector<std::string> states_;
    int fanNum_ = 0;
    cv::Point2f center_;
    bool isImshow_ = true;
    cv::Mat frame_;
    int count_ = 0;
    std::string lastFailureReason_;
};

}

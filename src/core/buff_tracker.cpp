#include "buff_tracker.hpp"

#include "angle_processor.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace gutcpp {

namespace {

double ApplyIoUType(const BBox& a, const BBox& b, IoUType type) {
    switch (type) {
        case IoUType::IoU:
            return IoU(a, b);
        case IoUType::GIoU:
            return GIoU(a, b);
        case IoUType::DIoU:
            return DIoU(a, b);
        case IoUType::CIoU:
            return CIoU(a, b);
    }
    return IoU(a, b);
}

}

BBox::BBox(double xminValue, double yminValue, double xmaxValue, double ymaxValue, int bboxId)
    : xmin(xminValue), ymin(yminValue), xmax(xmaxValue), ymax(ymaxValue), id(bboxId) {}

double BBox::intersectionArea(const BBox& other) const {
    const double xMax = std::min(xmax, other.xmax);
    const double yMax = std::min(ymax, other.ymax);
    const double xMin = std::max(xmin, other.xmin);
    const double yMin = std::max(ymin, other.ymin);
    const BBox crossBox(xMin, yMin, xMax, yMax);
    if (crossBox.width() <= 0.0 || crossBox.height() <= 0.0) {
        return 0.0;
    }
    return crossBox.area();
}

double BBox::unionArea(const BBox& other) const {
    return area() + other.area() - intersectionArea(other);
}

double BBox::iou(const BBox& other) const {
    return intersectionArea(other) / (unionArea(other) + 1e-6);
}

BBox BBox::boundOf(const BBox& other) const {
    return {std::min(xmin, other.xmin), std::min(ymin, other.ymin), std::max(xmax, other.xmax),
            std::max(ymax, other.ymax)};
}

double BBox::centerDistance(const BBox& other) const {
    return EuclideanDistance(center2f(), other.center2f());
}

double BBox::boundDiagonalDistance(const BBox& other) const {
    const BBox bound = boundOf(other);
    return EuclideanDistance(cv::Point2f(static_cast<float>(bound.xmin), static_cast<float>(bound.ymin)),
                             cv::Point2f(static_cast<float>(bound.xmax), static_cast<float>(bound.ymax)));
}

cv::Point2f BBox::center2f() const {
    return {static_cast<float>((xmin + xmax) / 2.0), static_cast<float>((ymin + ymax) / 2.0)};
}

cv::Point BBox::center2i() const {
    return {static_cast<int>((xmin + xmax) / 2.0), static_cast<int>((ymin + ymax) / 2.0)};
}

double BBox::area() const {
    return width() * height();
}

double BBox::width() const {
    return xmax - xmin;
}

double BBox::height() const {
    return ymax - ymin;
}

BBox BBox::createNewBBoxByCenter(const cv::Point2f& center) const {
    const int newXMin = static_cast<int>(center.x - width() / 2.0);
    const int newYMin = static_cast<int>(center.y - height() / 2.0);
    const int newXMax = static_cast<int>(center.x + width() / 2.0);
    const int newYMax = static_cast<int>(center.y + height() / 2.0);
    return BBox(newXMin, newYMin, newXMax, newYMax, id);
}

cv::Point BBox::p1i() const {
    return {static_cast<int>(xmin), static_cast<int>(ymin)};
}

cv::Point BBox::p2i() const {
    return {static_cast<int>(xmax), static_cast<int>(ymax)};
}

RotationRectangle::RotationRectangle(const std::vector<cv::Point2f>& rawPoints, const cv::Point2f& rBoxCenter) {
    if (rawPoints.size() != 4) {
        throw std::runtime_error("RotationRectangle requires exactly 4 points");
    }

    std::vector<std::pair<cv::Point2f, double>> rankedPoints;
    rankedPoints.reserve(4);
    for (const auto& point : rawPoints) {
        rankedPoints.emplace_back(point, EuclideanDistance(point, rBoxCenter));
    }
    std::sort(rankedPoints.begin(), rankedPoints.end(), [](const auto& lhs, const auto& rhs) {
        return lhs.second > rhs.second;
    });

    p1 = rankedPoints[0].first;
    p2 = rankedPoints[1].first;
    rankedPoints[2].second = EuclideanDistance(p1, rankedPoints[2].first);
    rankedPoints[3].second = EuclideanDistance(p1, rankedPoints[3].first);
    if (rankedPoints[2].second > rankedPoints[3].second) {
        p3 = rankedPoints[2].first;
        p4 = rankedPoints[3].first;
    } else {
        p3 = rankedPoints[3].first;
        p4 = rankedPoints[2].first;
    }

    points = {p1, p2, p3, p4};
    top = lineCenter(p1, p2);
    btm = lineCenter(p3, p4);
    disTop = EuclideanDistance(top, rBoxCenter);
    disBtm = EuclideanDistance(btm, rBoxCenter);
}

cv::Point2f RotationRectangle::lineCenter(const cv::Point2f& first, const cv::Point2f& second) {
    return {(std::min(first.x, second.x) + std::max(first.x, second.x)) / 2.0f,
            (std::min(first.y, second.y) + std::max(first.y, second.y)) / 2.0f};
}

cv::Point2f RotationRectangle::center2f() const {
    auto lineParameters = [](const cv::Point2f& first, const cv::Point2f& second) -> std::pair<double, double> {
        const double x1 = first.x;
        const double y1 = first.y;
        const double x2 = second.x;
        const double y2 = second.y;
        const double dy = y2 - y1;
        if (std::fabs(dy) < 1e-9) {
            return {std::numeric_limits<double>::infinity(), 0.0};
        }
        const double k = (x2 - x1) / dy;
        const double b = y1 - k * x1;
        return {k, b};
    };

    const auto [k13, b13] = lineParameters(p1, p3);
    const auto [k24, b24] = lineParameters(p2, p4);
    if (!std::isfinite(k13) || !std::isfinite(k24) || std::fabs(k13 - k24) < 1e-9) {
        cv::Point2f sum(0.0f, 0.0f);
        for (const auto& point : points) {
            sum += point;
        }
        return sum * (1.0f / static_cast<float>(points.size()));
    }

    cv::Mat coefficients = (cv::Mat_<double>(2, 2) << 1.0, -k13, 1.0, -k24);
    cv::Mat constants = (cv::Mat_<double>(2, 1) << b13, b24);
    cv::Mat solution;
    if (!cv::solve(coefficients, constants, solution, cv::DECOMP_SVD)) {
        cv::Point2f sum(0.0f, 0.0f);
        for (const auto& point : points) {
            sum += point;
        }
        return sum * (1.0f / static_cast<float>(points.size()));
    }
    return {static_cast<float>(solution.at<double>(0, 0)), static_cast<float>(solution.at<double>(1, 0))};
}

cv::Point RotationRectangle::center2i() const {
    const cv::Point2f center = center2f();
    return {static_cast<int>(center.x), static_cast<int>(center.y)};
}

double RotationRectangle::width() const {
    return EuclideanDistance(p1, p2);
}

double RotationRectangle::height() const {
    return EuclideanDistance(p1, p4);
}

double RotationRectangle::area() const {
    return width() * height();
}

double IoU(const BBox& a, const BBox& b) {
    return a.iou(b);
}

double GIoU(const BBox& a, const BBox& b) {
    const double boundArea = a.boundOf(b).area();
    const double unionArea = a.unionArea(b);
    return IoU(a, b) - (boundArea - unionArea) / boundArea;
}

double DIoU(const BBox& a, const BBox& b) {
    const double d = a.centerDistance(b);
    const double c = a.boundDiagonalDistance(b);
    return IoU(a, b) - (d * d) / (c * c);
}

double CIoU(const BBox& a, const BBox& b) {
    const double v = 4.0 / (CV_PI * CV_PI) *
                     std::pow(std::atan(a.width() / a.height()) - std::atan(b.width() / b.height()), 2.0);
    const double iou = IoU(a, b);
    const double alpha = v / (1.0 - iou + v);
    return DIoU(a, b) - alpha * v;
}

std::vector<TargetStruct> CompareByIoU(const BBox& lastBox, const std::vector<BBox>& boxes, IoUType type) {
    std::vector<TargetStruct> ious;
    ious.reserve(boxes.size());
    for (const auto& box : boxes) {
        ious.push_back({box, ApplyIoUType(lastBox, box, type)});
    }
    std::sort(ious.begin(), ious.end(), [](const TargetStruct& lhs, const TargetStruct& rhs) {
        return lhs.iou > rhs.iou;
    });
    return ious;
}

F_BuffTracker::F_BuffTracker(BBox fanBladeBox, BBox rBox, Parameter parameter, bool isImshow)
    : parameter_(std::move(parameter)),
      fanBladeBox_(std::move(fanBladeBox)),
      rBox_(std::move(rBox)),
      fanBladeList_(5),
      radius_(rBox_.centerDistance(fanBladeBox_)),
      states_({"target", "unlighted", "unlighted", "unlighted", "unlighted"}),
      center_(rBox_.center2f()),
      isImshow_(isImshow) {
    for (auto& fanBlade : fanBladeList_) {
        fanBlade.bbox = BBox(0, 0, 0, 0);
        fanBlade.rtnRect = makeDummyRotationRect(rBox_.center2f());
    }
}

RotationRectangle F_BuffTracker::makeDummyRotationRect(const cv::Point2f& rBoxCenter) {
    const std::vector<cv::Point2f> points = {
        {rBoxCenter.x - 1.0f, rBoxCenter.y - 1.0f},
        {rBoxCenter.x + 1.0f, rBoxCenter.y - 1.0f},
        {rBoxCenter.x + 1.0f, rBoxCenter.y + 1.0f},
        {rBoxCenter.x - 1.0f, rBoxCenter.y + 1.0f},
    };
    return RotationRectangle(points, rBoxCenter);
}

void F_BuffTracker::safeImshow(const std::string& windowName, const cv::Mat& image) {
    if (!image.empty()) {
        cv::imshow(windowName, image);
    }
}

BBox F_BuffTracker::pointsToBBox(const std::vector<cv::Point2f>& points) {
    double minX = std::numeric_limits<double>::max();
    double minY = std::numeric_limits<double>::max();
    double maxX = std::numeric_limits<double>::lowest();
    double maxY = std::numeric_limits<double>::lowest();
    for (const auto& point : points) {
        minX = std::min(minX, static_cast<double>(point.x));
        minY = std::min(minY, static_cast<double>(point.y));
        maxX = std::max(maxX, static_cast<double>(point.x));
        maxY = std::max(maxY, static_cast<double>(point.y));
    }
    return {minX, minY, maxX, maxY};
}

cv::Mat F_BuffTracker::getMaskByHSVThreshold(const cv::Mat& frame) const {
    cv::Mat hsv;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
    cv::Mat mask;
    cv::inRange(hsv, parameter_.hsv.lowerLimit, parameter_.hsv.upperLimit, mask);
    if (parameter_.kernel > 0) {
        const cv::Mat kernel = cv::Mat::ones(parameter_.kernel, parameter_.kernel, CV_8U);
        cv::dilate(mask, mask, kernel, cv::Point(-1, -1), 1);
    }
    return mask;
}

std::optional<std::vector<FanBlade>> F_BuffTracker::getFanBlade(cv::Mat& mask) {
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    std::vector<FanBlade> fanBladeList;
    std::vector<FanBlade> realFanBladeList;

    for (const auto& contour : contours) {
        cv::RotatedRect minRect = cv::minAreaRect(contour);
        std::array<cv::Point2f, 4> rectPointsArray{};
        minRect.points(rectPointsArray.data());
        const std::vector<cv::Point2f> rectPoints(rectPointsArray.begin(), rectPointsArray.end());
        RotationRectangle rtnRect(rectPoints, rBox_.center2f());
        if (0.4 * radius_ < rtnRect.disBtm && rtnRect.disBtm < rtnRect.disTop && rtnRect.disTop < 1.5 * radius_ &&
            rtnRect.area() > 2.0 * rBox_.area()) {
            const cv::Rect rect = cv::boundingRect(contour);
            if (isImshow_) {
                cv::rectangle(mask, rect, cv::Scalar(255, 255, 255), 3);
            }
            fanBladeList.push_back({BBox(rect.x, rect.y, rect.x + rect.width, rect.y + rect.height), rtnRect});
        }
    }

    if (isImshow_) {
        safeImshow("mask__", mask);
    }

    if (fanBladeList_[0].bbox.area() == 0.0 && fanBladeList.size() == 1U) {
        fanBladeList[0].bbox.id = 0;
        return fanBladeList;
    }
    if (fanBladeList_[0].bbox.area() == 0.0 && fanBladeList.size() != 1U) {
        lastFailureReason_ = "Expected exactly one initial lit fan blade contour, got " +
                             std::to_string(fanBladeList.size());
        return std::nullopt;
    }

    std::vector<FanBlade> correctFanBlade;
    correctFanBlade.reserve(fanBladeList_.size());
    for (const auto& fanBlade : fanBladeList_) {
        const cv::Point2f tempXY = fanBlade.bbox.center2f() - center_ + rBox_.center2f();
        BBox tempBox = fanBlade.bbox.createNewBBoxByCenter(tempXY);
        std::vector<cv::Point2f> tempPoints;
        tempPoints.reserve(fanBlade.rtnRect.points.size());
        for (const auto& point : fanBlade.rtnRect.points) {
            tempPoints.push_back(point - center_ + rBox_.center2f());
        }
        correctFanBlade.push_back({tempBox, RotationRectangle(tempPoints, rBox_.center2f())});
        if (isImshow_) {
            cv::rectangle(frame_, tempBox.p1i(), tempBox.p2i(), cv::Scalar(255, 0, 0), 3);
            std::ostringstream label;
            label << "id = " << tempBox.id << " | lastFrame";
            cv::putText(frame_, label.str(), tempBox.p1i() - cv::Point(30, 30), cv::FONT_HERSHEY_SIMPLEX, 0.75,
                        cv::Scalar(255, 0, 0), 2);
        }
    }

    for (const auto& lastFan : correctFanBlade) {
        const BBox& box = lastFan.bbox;
        std::vector<FanBlade> tempList;
        for (const auto& fan : fanBladeList) {
            if (IoU(box, fan.bbox) > 0.0) {
                tempList.push_back(fan);
            }
        }
        if (tempList.size() == 1U) {
            tempList[0].bbox.id = box.id;
            realFanBladeList.push_back(tempList[0]);
        } else if (tempList.size() >= 2U) {
            std::array<std::vector<std::pair<cv::Point2f, double>>, 4> tempPoints;
            for (const auto& rtn : tempList) {
                tempPoints[0].push_back({rtn.rtnRect.p1, EuclideanDistance(rtn.rtnRect.p1, rBox_.center2f())});
                tempPoints[1].push_back({rtn.rtnRect.p2, EuclideanDistance(rtn.rtnRect.p2, rBox_.center2f())});
                tempPoints[2].push_back({rtn.rtnRect.p3, EuclideanDistance(rtn.rtnRect.p3, rBox_.center2f())});
                tempPoints[3].push_back({rtn.rtnRect.p4, EuclideanDistance(rtn.rtnRect.p4, rBox_.center2f())});
            }

            const cv::Point2f p1 = std::max_element(tempPoints[0].begin(), tempPoints[0].end(),
                                                    [](const auto& lhs, const auto& rhs) { return lhs.second < rhs.second; })
                                      ->first;
            const cv::Point2f p2 = std::max_element(tempPoints[1].begin(), tempPoints[1].end(),
                                                    [](const auto& lhs, const auto& rhs) { return lhs.second < rhs.second; })
                                      ->first;
            const cv::Point2f p3 = std::min_element(tempPoints[2].begin(), tempPoints[2].end(),
                                                    [](const auto& lhs, const auto& rhs) { return lhs.second < rhs.second; })
                                      ->first;
            const cv::Point2f p4 = std::min_element(tempPoints[3].begin(), tempPoints[3].end(),
                                                    [](const auto& lhs, const auto& rhs) { return lhs.second < rhs.second; })
                                      ->first;

            BBox bbox = pointsToBBox({p1, p2, p3, p4});
            bbox.id = box.id;
            realFanBladeList.push_back({bbox, RotationRectangle({p1, p2, p3, p4}, rBox_.center2f())});
        }
    }

    if (realFanBladeList.size() == 1U) {
        realFanBladeList[0].bbox.id = 0;
        states_[0] = "target";
        for (int innerIndex = 1; innerIndex < 5; ++innerIndex) {
            states_[static_cast<std::size_t>(innerIndex)] = "unlighted";
        }
    } else if (static_cast<int>(realFanBladeList.size()) > fanNum_) {
        for (const auto& fan : realFanBladeList) {
            const int id = fan.bbox.id;
            if (id >= 0 && id < 5 && states_[static_cast<std::size_t>(id)] == "target") {
                states_[static_cast<std::size_t>(id)] = "shot";
            }
        }
        for (const auto& fan : realFanBladeList) {
            const int id = fan.bbox.id;
            if (id >= 0 && id < 5 && states_[static_cast<std::size_t>(id)] == "unlighted") {
                states_[static_cast<std::size_t>(id)] = "target";
                break;
            }
        }
    }
    return realFanBladeList;
}

bool F_BuffTracker::mayBeTarget(double width, double height, bool flag) const {
    if (flag) {
        return std::min(width * height, rBox_.area()) / std::max(width * height, rBox_.area()) > parameter_.maybeTarget.area &&
               std::min(width, rBox_.width()) / std::max(width, rBox_.width()) > parameter_.maybeTarget.width &&
               std::min(height, rBox_.height()) / std::max(height, rBox_.height()) > parameter_.maybeTarget.height;
    }
    return true;
}

std::vector<BBox> F_BuffTracker::getAlternateBoxes(const cv::Mat& mask, bool flag) const {
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    std::vector<BBox> boxes;
    boxes.reserve(contours.size());
    for (const auto& contour : contours) {
        const cv::Rect rect = cv::boundingRect(contour);
        const double distance = EuclideanDistance(
            cv::Point2f(static_cast<float>(rect.x + rect.width / 2.0), static_cast<float>(rect.y + rect.height / 2.0)),
            rBox_.center2f());
        if (mayBeTarget(static_cast<double>(rect.width), static_cast<double>(rect.height), flag) && distance < radius_ * 2.0) {
            boxes.emplace_back(rect.x, rect.y, rect.x + rect.width, rect.y + rect.height);
        }
    }
    return boxes;
}

bool F_BuffTracker::update(cv::Mat& frame, bool isOpenMaybeTarget) {
    if (isImshow_) {
        frame_ = frame;
    }
    std::vector<int> lightedFanBladeIdList = {0};
    cv::Mat mask = getMaskByHSVThreshold(frame);
    if (isImshow_) {
        safeImshow("maskI", mask);
    }
    const std::vector<BBox> boxes = getAlternateBoxes(mask, isOpenMaybeTarget);
    const std::vector<TargetStruct> boxAndIoU = CompareByIoU(rBox_, boxes, IoUType::CIoU);
    if (!boxAndIoU.empty() && boxAndIoU[0].iou > -1.0) {
        rBox_ = boxAndIoU[0].box;
    } else {
        lastFailureReason_ = "Center R candidate not found";
        return false;
    }

    cv::circle(mask, rBox_.center2i(), static_cast<int>(radius_ * parameter_.insideRate), cv::Scalar(0, 0, 0), -1);
    cv::circle(mask, rBox_.center2i(), static_cast<int>(radius_ * parameter_.outsideRate), cv::Scalar(0, 0, 0), 3);

    const std::optional<std::vector<FanBlade>> fanBladeList = getFanBlade(mask);
    cv::waitKey(1);
    center_ = rBox_.center2f();
    ++count_;
    if (!fanBladeList.has_value()) {
        if (lastFailureReason_.empty()) {
            lastFailureReason_ = "Fan blade extraction failed";
        }
        return false;
    }

    lastFailureReason_.clear();

    fanNum_ = static_cast<int>(fanBladeList->size());
    for (const auto& fan : fanBladeList.value()) {
        const BBox& box = fan.bbox;
        if (box.id != 0) {
            lightedFanBladeIdList.push_back(box.id);
        }
        fanBladeList_[static_cast<std::size_t>(box.id)].bbox = box;
        if (states_[static_cast<std::size_t>(box.id)] == "target") {
            fanBladeBox_ = box;
        }
        if (isImshow_) {
            std::ostringstream label;
            label << "id = " << box.id << " | " << states_[static_cast<std::size_t>(box.id)];
            cv::rectangle(frame, box.p1i(), box.p2i(), cv::Scalar(0, 0, 255), 2);
            cv::putText(frame, label.str(), box.p1i(), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0, 0, 255), 2);
            cv::rectangle(frame, rBox_.p1i(), rBox_.p2i(), cv::Scalar(0, 0, 255), 3);
            cv::putText(frame, "R", rBox_.p1i(), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0, 0, 255), 2);
            cv::putText(frame, "press Q to quit", cv::Point(20, 20), cv::FONT_HERSHEY_SIMPLEX, 0.75,
                        cv::Scalar(0, 0, 255), 2);
        }
    }

    for (int index = 0; index < 5; ++index) {
        if (std::find(lightedFanBladeIdList.begin(), lightedFanBladeIdList.end(), index) != lightedFanBladeIdList.end()) {
            continue;
        }
        BBox bbox = fanBladeBox_.createNewBBoxByCenter(
            Rotate(2.0 * CV_PI / 5.0 * static_cast<double>(index), fanBladeList_[0].bbox.center2f() - rBox_.center2f()) +
            rBox_.center2f());
        bbox.id = index;
        fanBladeList_[static_cast<std::size_t>(index)].bbox = bbox;
    }
    return true;
}

const BBox& F_BuffTracker::fanBladeBox() const {
    return fanBladeBox_;
}

const BBox& F_BuffTracker::rBox() const {
    return rBox_;
}

double F_BuffTracker::radius() const {
    return radius_;
}

const std::string& F_BuffTracker::lastFailureReason() const {
    return lastFailureReason_;
}

}

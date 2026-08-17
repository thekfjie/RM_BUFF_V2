#include "standalone_visual_tools.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <unordered_set>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

namespace gutcpp::standalone {

void SafeImshow(const std::string& windowName, const cv::Mat& image) {
    if (!image.empty()) {
        static std::unordered_set<std::string> initializedWindows;
        if (initializedWindows.find(windowName) == initializedWindows.end()) {
            const cv::Size size = [&]() {
                const int width = image.cols;
                const int height = image.rows;
                constexpr int maxWidth = 1600;
                constexpr int maxHeight = 900;
                if (width <= maxWidth && height <= maxHeight) {
                    return cv::Size(width, height);
                }
                const double scale = std::min(static_cast<double>(maxWidth) / static_cast<double>(width),
                                              static_cast<double>(maxHeight) / static_cast<double>(height));
                return cv::Size(std::max(1, static_cast<int>(width * scale)),
                                std::max(1, static_cast<int>(height * scale)));
            }();
            EnsureResizableWindow(windowName, size);
            initializedWindows.insert(windowName);
        }
        cv::imshow(windowName, image);
    }
}

void EnsureResizableWindow(const std::string& windowName, const cv::Size& size) {
    cv::namedWindow(windowName, cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO);
    cv::resizeWindow(windowName, size.width, size.height);
}

int ClampInt(int value, int low, int high) {
    return std::max(low, std::min(value, high));
}

cv::Size FitWindowSize(const cv::Mat& image, int maxWidth = 1600, int maxHeight = 900) {
    const int width = image.cols;
    const int height = image.rows;
    if (width <= maxWidth && height <= maxHeight) {
        return cv::Size(width, height);
    }
    const double scale = std::min(static_cast<double>(maxWidth) / static_cast<double>(width),
                                  static_cast<double>(maxHeight) / static_cast<double>(height));
    return cv::Size(std::max(1, static_cast<int>(width * scale)), std::max(1, static_cast<int>(height * scale)));
}

cv::Rect SelectRoiFitted(const std::string& windowName, const cv::Mat& image) {
    EnsureResizableWindow(windowName, FitWindowSize(image));
    SafeImshow(windowName, image);
    return cv::selectROI(windowName, image, false, false);
}

TuneControls ControlsFromParameter(const Parameter& parameter) {
    TuneControls controls;
    controls.lh = ClampInt(static_cast<int>(parameter.hsv.lowerLimit[0]), 0, 255);
    controls.ls = ClampInt(static_cast<int>(parameter.hsv.lowerLimit[1]), 0, 255);
    controls.lv = ClampInt(static_cast<int>(parameter.hsv.lowerLimit[2]), 0, 255);
    controls.uh = ClampInt(static_cast<int>(parameter.hsv.upperLimit[0]), 0, 255);
    controls.us = ClampInt(static_cast<int>(parameter.hsv.upperLimit[1]), 0, 255);
    controls.uv = ClampInt(static_cast<int>(parameter.hsv.upperLimit[2]), 0, 255);
    controls.kernel = ClampInt(parameter.kernel, 0, 10);
    controls.outside = ClampInt(static_cast<int>(parameter.outsideRate * 100.0), 0, 200);
    controls.inside = ClampInt(static_cast<int>(parameter.insideRate * 100.0), 0, 100);
    return controls;
}

void CreateTuneTrackbars(TuneControls& controls) {
    EnsureResizableWindow("Tracking", cv::Size(800, 600));
    cv::createTrackbar("LH", "Tracking", &controls.lh, 255);
    cv::createTrackbar("LS", "Tracking", &controls.ls, 255);
    cv::createTrackbar("LV", "Tracking", &controls.lv, 255);
    cv::createTrackbar("UH", "Tracking", &controls.uh, 255);
    cv::createTrackbar("US", "Tracking", &controls.us, 255);
    cv::createTrackbar("UV", "Tracking", &controls.uv, 255);
    cv::createTrackbar("kernel", "Tracking", &controls.kernel, 10);
    cv::createTrackbar("outside", "Tracking", &controls.outside, 200);
    cv::createTrackbar("inside", "Tracking", &controls.inside, 100);
}

cv::Mat BuildTuneMask(const cv::Mat& frame, const TuneControls& controls) {
    cv::Mat hsv;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
    cv::Mat mask;
    cv::inRange(hsv, cv::Scalar(controls.lh, controls.ls, controls.lv), cv::Scalar(controls.uh, controls.us, controls.uv),
                mask);
    if (controls.kernel > 0) {
        cv::dilate(mask, mask, cv::Mat::ones(controls.kernel, controls.kernel, CV_8U), cv::Point(-1, -1), 1);
    }
    return mask;
}

bool ReadExactFrame(cv::VideoCapture& capture, int targetFrame, cv::Mat& frame) {
    if (targetFrame < 1) {
        return false;
    }
    capture.set(cv::CAP_PROP_POS_FRAMES, static_cast<double>(targetFrame - 1));
    return capture.read(frame);
}

cv::Mat BrowseTuneFrame(cv::VideoCapture& capture, int& selectedFrameIndex, double previewSpeed) {
    EnsureResizableWindow("frame", cv::Size(1280, 720));
    cv::Mat frame;
    if (!capture.read(frame)) {
        throw std::runtime_error("Failed to read first frame for tuning");
    }

    double sourceFps = capture.get(cv::CAP_PROP_FPS);
    if (sourceFps <= 1e-3) {
        sourceFps = 30.0;
    }
    const int delayMs = std::max(1, static_cast<int>(std::round(1000.0 / sourceFps / std::max(0.25, std::min(previewSpeed, 8.0)))));
    int frameIndex = 1;
    while (true) {
        cv::Mat display = frame.clone();
        cv::putText(display, "Tuner Preview: SPACE freeze current frame, q quit", cv::Point(20, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0, 255, 0), 2);
        cv::putText(display, "frame = " + std::to_string(frameIndex), cv::Point(20, 65), cv::FONT_HERSHEY_SIMPLEX,
                    0.75, cv::Scalar(0, 255, 0), 2);
        cv::putText(display, "speed = " + std::to_string(previewSpeed) + "x", cv::Point(20, 100),
                    cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0, 255, 0), 2);
        SafeImshow("frame", display);
        const int key = cv::waitKey(delayMs);
        if (key == 'q' || key == 'Q' || key == 27) {
            throw std::runtime_error("Tune frame selection cancelled");
        }
        if (key == ' ') {
            selectedFrameIndex = frameIndex;
            return frame;
        }
        cv::Mat nextFrame;
        if (!capture.read(nextFrame)) {
            selectedFrameIndex = frameIndex;
            return frame;
        }
        frame = nextFrame;
        ++frameIndex;
    }
}

void RunTuneMode(const Options& options, Parameter parameter, const fs::path& resolvedParameterPath,
                 const fs::path& videoPath) {
    cv::VideoCapture capture(videoPath.string());
    if (!capture.isOpened()) {
        throw std::runtime_error("Failed to open video for tuning: " + videoPath.string());
    }

    cv::Mat tuningFrame;
    int selectedFrameIndex = parameter.start;
    if (options.tuneFrame.has_value()) {
        selectedFrameIndex = options.tuneFrame.value();
        if (!ReadExactFrame(capture, selectedFrameIndex, tuningFrame)) {
            throw std::runtime_error("Failed to read tune frame: " + std::to_string(selectedFrameIndex));
        }
    } else {
        tuningFrame = BrowseTuneFrame(capture, selectedFrameIndex, options.tunePreviewSpeed);
    }

    const cv::Rect rRect = options.rBoxOverride.has_value() ? options.rBoxOverride.value() : SelectRoiFitted("roi", tuningFrame);
    const cv::Rect fanRect = options.fanBoxOverride.has_value() ? options.fanBoxOverride.value()
                                                                : SelectRoiFitted("roi2", tuningFrame);
    const BBox rBox = RoiToBBox(rRect);
    const BBox fanBladeBox = RoiToBBox(fanRect);
    const double radius = rBox.centerDistance(fanBladeBox);

    TuneControls controls = ControlsFromParameter(parameter);
    CreateTuneTrackbars(controls);
    EnsureResizableWindow("frame", cv::Size(1280, 720));
    EnsureResizableWindow("mask", cv::Size(960, 720));
    EnsureResizableWindow("res", cv::Size(960, 720));

    std::cout << "Tune mode: adjust LH/LS/LV/UH/US/UV/kernel/outside/inside, press q to save, ESC to cancel." << std::endl;

    while (true) {
        cv::Mat frameView = tuningFrame.clone();
        cv::Mat mask = BuildTuneMask(tuningFrame, controls);
        cv::Mat res;
        cv::bitwise_and(tuningFrame, tuningFrame, res, mask);

        cv::circle(frameView, rBox.center2i(), static_cast<int>(radius * static_cast<double>(controls.inside) / 100.0),
                   cv::Scalar(255, 0, 0), -1);
        cv::circle(frameView, rBox.center2i(), static_cast<int>(radius * static_cast<double>(controls.outside) / 100.0),
                   cv::Scalar(255, 0, 0), 3);
        cv::circle(mask, rBox.center2i(), static_cast<int>(radius * static_cast<double>(controls.inside) / 100.0),
                   cv::Scalar(0, 0, 0), -1);
        cv::circle(mask, rBox.center2i(), static_cast<int>(radius * static_cast<double>(controls.outside) / 100.0),
                   cv::Scalar(255, 255, 255), 3);

        SafeImshow("frame", frameView);
        SafeImshow("mask", mask);
        SafeImshow("res", res);

        const int key = cv::waitKey(1);
        if (key < 0) {
            continue;
        }
        if (key == 27) {
            std::cout << "Tune cancelled" << std::endl;
            return;
        }
        if (key == 'q' || key == 'Q') {
            parameter.parameterPath = resolvedParameterPath;
            parameter.hsv.lowerLimit = cv::Scalar(controls.lh, controls.ls, controls.lv);
            parameter.hsv.upperLimit = cv::Scalar(controls.uh, controls.us, controls.uv);
            parameter.kernel = controls.kernel;
            parameter.outsideRate = static_cast<double>(controls.outside) / 100.0;
            parameter.insideRate = static_cast<double>(controls.inside) / 100.0;
            parameter.start = selectedFrameIndex;
            if (options.videoPathOverride.has_value()) {
                parameter.videoRelativePath = ResolveRawPath(options.pythonRoot, options.videoPathOverride.value()).string();
            }
            SaveParameter(parameter);
            std::cout << "Saved tuned parameter to: " << resolvedParameterPath.string() << std::endl;
            return;
        }
    }
}

fs::path AngleDumpPathFor(const Parameter& parameter) {
    fs::path baseName = parameter.parameterPath.parent_path().filename();
    if (baseName.empty()) {
        baseName = parameter.parameterPath.stem();
    }
    return fs::current_path() / (baseName.string() + ".txt");
}

void WriteAnglesFile(const std::vector<double>& angles, const Parameter& parameter) {
    const fs::path outputPath = AngleDumpPathFor(parameter);
    std::ofstream output(outputPath);
    if (!output.is_open()) {
        throw std::runtime_error("Failed to open angle dump file: " + outputPath.string());
    }
    for (double angle : angles) {
        output << angle << '\n';
    }
}

cv::Mat BuildAnglesPlotImage(const std::vector<double>& angles) {
    constexpr int width = 1280;
    constexpr int height = 720;
    constexpr int leftMargin = 80;
    constexpr int rightMargin = 40;
    constexpr int topMargin = 40;
    constexpr int bottomMargin = 80;

    cv::Mat plot(height, width, CV_8UC3, cv::Scalar(255, 255, 255));
    cv::rectangle(plot, cv::Rect(0, 0, width, height), cv::Scalar(255, 255, 255), cv::FILLED);
    cv::line(plot, cv::Point(leftMargin, height - bottomMargin), cv::Point(width - rightMargin, height - bottomMargin),
             cv::Scalar(0, 0, 0), 2);
    cv::line(plot, cv::Point(leftMargin, topMargin), cv::Point(leftMargin, height - bottomMargin), cv::Scalar(0, 0, 0), 2);
    cv::putText(plot, "angle", cv::Point(20, topMargin), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 0), 2);
    cv::putText(plot, "frame", cv::Point(width - 120, height - 20), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 0), 2);

    if (angles.empty()) {
        cv::putText(plot, "No angles collected", cv::Point(120, height / 2), cv::FONT_HERSHEY_SIMPLEX, 1.0,
                    cv::Scalar(0, 0, 255), 2);
        return plot;
    }

    const auto [minIt, maxIt] = std::minmax_element(angles.begin(), angles.end());
    double minAngle = *minIt;
    double maxAngle = *maxIt;
    if (std::fabs(maxAngle - minAngle) < 1e-9) {
        minAngle -= 1.0;
        maxAngle += 1.0;
    }

    const int plotWidth = width - leftMargin - rightMargin;
    const int plotHeight = height - topMargin - bottomMargin;
    std::vector<cv::Point> points;
    points.reserve(angles.size());
    for (std::size_t index = 0; index < angles.size(); ++index) {
        const double xRatio = (angles.size() == 1U) ? 0.0 : static_cast<double>(index) / static_cast<double>(angles.size() - 1U);
        const double yRatio = (angles[index] - minAngle) / (maxAngle - minAngle);
        const int x = leftMargin + static_cast<int>(xRatio * static_cast<double>(plotWidth));
        const int y = topMargin + plotHeight - static_cast<int>(yRatio * static_cast<double>(plotHeight));
        points.emplace_back(x, y);
    }

    for (int tick = 0; tick <= 5; ++tick) {
        const double ratio = static_cast<double>(tick) / 5.0;
        const int y = topMargin + plotHeight - static_cast<int>(ratio * static_cast<double>(plotHeight));
        const double value = minAngle + ratio * (maxAngle - minAngle);
        cv::line(plot, cv::Point(leftMargin - 8, y), cv::Point(leftMargin, y), cv::Scalar(0, 0, 0), 1);
        cv::putText(plot, cv::format("%.3f", value), cv::Point(5, y + 5), cv::FONT_HERSHEY_SIMPLEX, 0.45,
                    cv::Scalar(0, 0, 0), 1);
    }

    if (points.size() == 1U) {
        cv::circle(plot, points.front(), 3, cv::Scalar(0, 128, 255), -1);
    } else {
        cv::polylines(plot, points, false, cv::Scalar(0, 128, 255), 2, cv::LINE_AA);
    }
    return plot;
}

void ShowAnglesPlot(const std::vector<double>& angles) {
    cv::Mat plot = BuildAnglesPlotImage(angles);
    EnsureResizableWindow("angles_plot", FitWindowSize(plot, 1280, 720));
    SafeImshow("angles_plot", plot);
    std::cout << "Angle plot: press any key to close." << std::endl;
    cv::waitKey(0);
}

BBox RoiToBBox(const cv::Rect& rect) {
    if (rect.width <= 0 || rect.height <= 0) {
        throw std::runtime_error("ROI must be non-empty");
    }
    return BBox(rect.x, rect.y, rect.x + rect.width, rect.y + rect.height);
}

cv::Rect BBoxToRect(const BBox& bbox) {
    const int width = std::max(1, static_cast<int>(std::lround(bbox.width())));
    const int height = std::max(1, static_cast<int>(std::lround(bbox.height())));
    return cv::Rect(static_cast<int>(std::lround(bbox.xmin)),
                    static_cast<int>(std::lround(bbox.ymin)),
                    width,
                    height);
}

int PreferredYoloSeedClassId(const std::string& color) {
    return (color == "red") ? 1 : 2;
}

} // namespace gutcpp::standalone

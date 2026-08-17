#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include "core/buff_pipeline.hpp"
#include "core/hsv_detector.hpp"
#include "core/parameter.hpp"
#include "core/yolo_detector.hpp"
#include "standalone_options.hpp"
#include "standalone_visual_tools.hpp"

namespace fs = std::filesystem;

int main(int argc, char** argv) {
    using namespace gutcpp;
    using namespace gutcpp::standalone;

    try {
        Options options = ParseArgs(argc, argv);
        if (options.promptPath) {
            PromptScenarioPath(options);
        }
        if (options.tune && !options.sourcePath.has_value()) {
            const fs::path defaultParameterPath = ResolveParameterPath(options.pythonRoot, options.parameterPath);
            const Parameter defaultParameter = gutcpp::LoadParameter(defaultParameterPath);
            const fs::path defaultVideoPath = ResolveVideoPath(defaultParameter, options.pythonRoot);
            PromptTuneVideoPathAndSpeed(options, defaultVideoPath);
        }
        ResolveScenarioSource(options);
        const fs::path resolvedParameterPath = ResolveParameterPath(options.pythonRoot, options.parameterPath);
        const Parameter parameter = gutcpp::LoadParameter(resolvedParameterPath);
        const fs::path videoPath = options.videoPathOverride.has_value()
                                      ? ResolveRawPath(options.pythonRoot, options.videoPathOverride.value())
                                      : ResolveVideoPath(parameter, options.pythonRoot);

        std::cout << "Using parameter: " << resolvedParameterPath.string() << std::endl;
        std::cout << "Using input source: " << videoPath.string() << std::endl;
        if (options.runConfigPath.has_value()) {
            std::cout << "Using run config: " << options.runConfigPath.value().string() << std::endl;
        }

        const bool useImageSequence = IsImageSequenceDirectory(videoPath);
        if (options.tune) {
            if (useImageSequence) {
                throw std::runtime_error("Tune mode does not support image sequence directories yet.");
            }
            RunTuneMode(options, parameter, resolvedParameterPath, videoPath);
            return 0;
        }

        std::unique_ptr<cv::VideoCapture> videoCapture;
        std::unique_ptr<ImageSequenceCapture> imageCapture;
        if (useImageSequence) {
            imageCapture = std::make_unique<ImageSequenceCapture>(videoPath);
            std::cout << "Using image sequence with " << imageCapture->size() << " frames" << std::endl;
        } else {
            videoCapture = std::make_unique<cv::VideoCapture>(videoPath.string());
            if (!videoCapture->isOpened()) {
                throw std::runtime_error("Failed to open video: " + videoPath.string());
            }
        }

        auto readNextFrame = [&](cv::Mat& nextFrame) -> bool {
            if (useImageSequence) {
                return imageCapture->read(nextFrame);
            }
            return videoCapture->read(nextFrame);
        };

        cv::Mat frame;
        if (!readNextFrame(frame)) {
            return -1;
        }

        std::vector<double> angles;
        std::vector<cv::Point2d> predictedPoints;
        int frameCount = 0;
        int interval = 0;
        int lostFrames = 0;
        int lastYoloAttemptFrame = -1000000;
        const fs::path logDir = fs::current_path() / "logs";
        fs::create_directories(logDir);
        const std::string modeStr = (options.moveMode == MoveMode::Small) ? "small" : "big";
        const fs::path logPath = logDir / (modeStr + "_" + options.color + "_predict.csv");
        std::ofstream logFile(logPath);
        logFile << "frame,observed_angle,raw_angle,delta_angle,pred_x,pred_y,debug_state\n";

        PipelineConfig pipeConfig;
        pipeConfig.moveMode = options.moveMode;
        pipeConfig.clockMode = ParseClockMode(options.color);
        pipeConfig.deltaT = options.deltaT;
        pipeConfig.freq = options.freq;
        pipeConfig.enableCompensation = parameter.enableCompensation;
        pipeConfig.compensationConfig.bulletSpeed = parameter.bulletSpeed;
        pipeConfig.compensationConfig.targetDistance = parameter.targetDistance;
        pipeConfig.compensationConfig.commLatencySec = parameter.commLatencySec;
        pipeConfig.compensationConfig.gimbalDelaySec = parameter.gimbalDelaySec;
        pipeConfig.compensationConfig.extraDelaySec = parameter.extraDelaySec;
        interval = static_cast<int>(static_cast<double>(options.freq) * options.deltaT);

        const std::string detType = !options.detectorOverride.empty()
            ? options.detectorOverride : parameter.detectorType;
        const std::string onnxPath = !options.onnxPathOverride.empty()
            ? options.onnxPathOverride : parameter.onnxModelPath;
        bool useYoloAssist = (detType == "yolo");
        const int preferredYoloClassId = PreferredYoloSeedClassId(options.color);

        std::unique_ptr<BuffPipeline> pipeline;
        std::unique_ptr<YoloDetector> yoloAssist;

        auto buildManualPipeline = [&](const cv::Mat& currentFrame) {
            const cv::Rect rRect = options.rBoxOverride.has_value() ? options.rBoxOverride.value()
                                                                    : SelectRoiFitted("roi", currentFrame);
            const cv::Rect fanRect = options.fanBoxOverride.has_value() ? options.fanBoxOverride.value()
                                                                        : SelectRoiFitted("roi2", currentFrame);
            auto hsvDet = std::make_unique<HsvDetector>(options.isImshow);
            auto newPipeline = std::make_unique<BuffPipeline>(std::move(hsvDet), pipeConfig);
            if (!newPipeline->initialize(currentFrame, parameter, rRect, fanRect)) {
                return false;
            }
            pipeline = std::move(newPipeline);
            lostFrames = 0;
            return true;
        };

        auto buildPipelineFromSeed = [&](const cv::Mat& currentFrame,
                                         const DetectionResult& seed,
                                         const std::string& reason) {
            auto hsvDet = std::make_unique<HsvDetector>(options.isImshow);
            auto newPipeline = std::make_unique<BuffPipeline>(std::move(hsvDet), pipeConfig);
            if (!newPipeline->initialize(currentFrame, parameter, BBoxToRect(seed.rBox), BBoxToRect(seed.fanBladeBox))) {
                std::cout << "Frame " << frameCount << ": YOLO " << reason
                          << " succeeded but HSV tracker init failed" << std::endl;
                return false;
            }
            pipeline = std::move(newPipeline);
            lostFrames = 0;
            std::cout << "Frame " << frameCount << ": YOLO " << reason
                      << " locked target, conf=" << seed.confidence
                      << " r=(" << seed.rBox.center2i().x << "," << seed.rBox.center2i().y << ")"
                      << " fan=(" << seed.fanBladeBox.center2i().x << "," << seed.fanBladeBox.center2i().y << ")"
                      << std::endl;
            return true;
        };

        auto tryYoloLock = [&](const cv::Mat& currentFrame, const std::string& reason) {
            if (!yoloAssist) {
                return false;
            }
            lastYoloAttemptFrame = frameCount;
            const std::optional<DetectionResult> seed =
                yoloAssist->detectTarget(currentFrame, preferredYoloClassId);
            if (!seed.has_value()) {
                std::cout << "Frame " << frameCount << ": YOLO " << reason << " miss" << std::endl;
                return false;
            }
            return buildPipelineFromSeed(currentFrame, seed.value(), reason);
        };

        if (useYoloAssist) {
            YoloDetectorConfig yoloCfg;
            yoloCfg.modelPath = onnxPath;
            yoloCfg.confidence = parameter.yoloConfidence;
            yoloCfg.nmsThreshold = parameter.yoloNmsThreshold;
            yoloCfg.inputWidth = parameter.yoloInputWidth;
            yoloCfg.inputHeight = parameter.yoloInputHeight;
            yoloCfg.refreshInterval = parameter.yoloRefreshInterval;

            yoloAssist = std::make_unique<YoloDetector>(yoloCfg, false);
            if (!yoloAssist->loadModel()) {
                std::cout << "YOLO model load failed, falling back to HSV manual mode" << std::endl;
                yoloAssist.reset();
                useYoloAssist = false;
            } else {
                std::cout << "YOLO assist active: model only initializes/relocks tracker" << std::endl;
            }
        }

        while (readNextFrame(frame)) {
            ++frameCount;
            std::cout << frameCount << std::endl;

            if (frameCount >= parameter.start) {
                if (!pipeline) {
                    if (useYoloAssist) {
                        if (tryYoloLock(frame, "init")) {
                            continue;
                        }
                        if (options.isImshow) {
                            SafeImshow("frame", frame);
                        }
                        const int key = cv::waitKey(1);
                        if (key == 'q' || key == 'Q') {
                            break;
                        }
                        continue;
                    }

                    if (!buildManualPipeline(frame)) {
                        throw std::runtime_error("Pipeline initialization failed");
                    }
                }

                const double timestampSeconds =
                    static_cast<double>(frameCount - 1) /
                    static_cast<double>(std::max(1, options.freq));
                PipelineOutput output = pipeline->processFrame(frame, timestampSeconds);
                if (output.rBox.area() == 0.0) {
                    if (useYoloAssist) {
                        ++lostFrames;
                        std::cout << "Frame " << frameCount << ": tracker lost target, missCount="
                                  << lostFrames << std::endl;
                        const bool shouldTryRelock =
                            (lostFrames >= options.yoloRelockAfterMisses) &&
                            ((frameCount - lastYoloAttemptFrame) >= options.yoloRelockIntervalFrames);
                        if (shouldTryRelock && tryYoloLock(frame, "relock")) {
                            continue;
                        }
                        if (options.isImshow) {
                            SafeImshow("frame", frame);
                        }
                        const int key = cv::waitKey(1);
                        if (key == 'q' || key == 'Q') {
                            break;
                        }
                        continue;
                    }
                    throw std::runtime_error("Tracker update failed");
                }
                lostFrames = 0;

                angles.push_back(output.observedAngle);

                if (output.predictionReady) {
                    const double x = output.predictedPoint.x;
                    const double y = output.predictedPoint.y;
                    predictedPoints.emplace_back(x, y);

                    logFile << frameCount << ","
                            << output.observedAngle << ","
                            << output.rawAngle << ","
                            << output.deltaAngle << ","
                            << x << "," << y << ","
                            << output.debugState << "\n";

                    cv::circle(frame, cv::Point(static_cast<int>(x), static_cast<int>(y)), 10, cv::Scalar(0, 255, 0), -1);
                    cv::putText(frame, "now predict", cv::Point(static_cast<int>(x), static_cast<int>(y)),
                                cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0, 0, 255), 2);

                    if (interval > 0 && static_cast<int>(predictedPoints.size()) >= interval) {
                        const cv::Point2d historyPoint =
                            predictedPoints[static_cast<std::size_t>(static_cast<int>(predictedPoints.size()) - interval)];
                        cv::circle(frame, cv::Point(static_cast<int>(historyPoint.x), static_cast<int>(historyPoint.y)), 10,
                                   cv::Scalar(0, 255, 0), -1);
                        cv::putText(frame, "0.2s before predict",
                                    cv::Point(static_cast<int>(historyPoint.x), static_cast<int>(historyPoint.y)),
                                    cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0, 0, 255), 2);
                    }
                }

                cv::rectangle(frame, output.fanBladeBox.p1i(), output.fanBladeBox.p2i(), cv::Scalar(0, 255, 0), 3);

                if (options.isImshow) {
                    SafeImshow("frame", frame);
                }
                const int key = cv::waitKey(1);
                if (key == 'q' || key == 'Q') {
                    break;
                }
            }
        }

        WriteAnglesFile(angles, parameter);
        logFile.flush();
        std::cout << "Prediction log: " << logPath.string() << std::endl;
        if (options.isImshow) {
            ShowAnglesPlot(angles);
        }
        std::cout << "DONE" << std::endl;
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << std::endl;
        return 1;
    }
}

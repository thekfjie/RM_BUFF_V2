#pragma once

#include <deque>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace gutcpp {

enum class MoveMode {
    Big,
    Small,
};

enum class ClockMode {
    Anticlockwise,
    Clockwise,
};

struct PredictionResult {
    bool ready = false;
    double deltaAngle = 0.0;
    double angularVelocity = 0.0;
    bool modelReady = false;
};

struct BigPredictorConfig {
    int omegaSearchSteps = 200;
    int fitUpdateStride = 5;
    std::size_t minInliers = 100;
    std::size_t maxSamples = 300;
    double minInlierRatio = 0.60;
    double inlierThreshold = 0.50;
    double minOmega = 1.884;
    double maxOmega = 2.000;
    double minAmplitude = 0.780;
    double maxAmplitude = 1.045;
    double maxAbsSpeed = 2.090;
    double maxObservationGap = 0.50;
    double maxPhaseJump = 0.80;
};

double EuclideanDistance(const cv::Point2f& p1, const cv::Point2f& p2);
cv::Point2f Rotate(double theta, const cv::Point2f& vector);
double trans(double x, double y);

class CircularQueue {
public:
    explicit CircularQueue(std::size_t queueCapacity);

    void push(double data);
    void pop();

    [[nodiscard]] std::size_t size() const;
    [[nodiscard]] bool isFull() const;
    [[nodiscard]] bool isEmpty() const;
    [[nodiscard]] double front() const;
    [[nodiscard]] double rear() const;
    [[nodiscard]] std::size_t rearIndex() const;

private:
    std::size_t capacity_;
    std::vector<double> circularQueue_;
    std::size_t frontIndex_ = 0;
    std::size_t rearIndex_ = 0;
};

class MovAvg {
public:
    explicit MovAvg(std::size_t windowSize = 7);
    double update(double data);

private:
    CircularQueue preSum_;
    CircularQueue dataQueue_;
    bool firstTime_ = true;
};

class FitStartDetect {
public:
    explicit FitStartDetect(std::size_t queueCapacity = 15);
    std::pair<bool, int> update(double data);

private:
    [[nodiscard]] bool isFlip() const;

    std::size_t queueCapacity_;
    CircularQueue queue_;
    std::vector<double> derivative_;
    int idx_ = 0;
    int flipCount_ = 0;
    int lim_ = 20;
    int count_ = 0;
};

class PredictorInterface {
public:
    virtual ~PredictorInterface() = default;
    virtual PredictionResult update(
        double data,
        double timestampSeconds = std::numeric_limits<double>::quiet_NaN()) = 0;
    virtual double predictDelta(double horizonSeconds) const = 0;
    virtual std::string debugState() const { return ""; }
};

class SmallPredictor final : public PredictorInterface {
public:
    SmallPredictor(double deltaT, int freq);
    PredictionResult update(
        double data,
        double timestampSeconds = std::numeric_limits<double>::quiet_NaN()) override;
    double predictDelta(double horizonSeconds) const override;
    std::string debugState() const override;

private:
    static constexpr double kSmallRuneSpeed = CV_PI / 3.0;  // 1/3π rad/s per rules
    double deltaAngle_ = 0.0;
    double defaultPredictionHorizon_ = 0.0;
    int warmupFrames_ = 0;
    int frameCount_ = 0;
    double firstAngle_ = 0.0;
    double direction_ = 0.0;
};

class BigPredictor final : public PredictorInterface {
public:
    BigPredictor(double deltaT, int freq, BigPredictorConfig config = {});
    PredictionResult update(
        double data,
        double timestampSeconds = std::numeric_limits<double>::quiet_NaN()) override;
    double predictDelta(double horizonSeconds) const override;
    std::string debugState() const override;

    struct FitState {
        double amplitude = 0.0;
        double omega = 0.0;
        double phase = 0.0;
        double offset = 0.0;
        double timeOrigin = 0.0;
        std::size_t inliers = 0;
        double rmse = std::numeric_limits<double>::infinity();
    };

    [[nodiscard]] const std::optional<FitState>& fitState() const { return fitState_; }
    [[nodiscard]] std::size_t sampleCount() const { return speedSamples_.size(); }

private:
    struct SpeedSample {
        double timestamp = 0.0;
        double velocity = 0.0;
    };

    static double velocityAt(double timestamp, const FitState& fitState);
    static double integrateVelocity(double startTimestamp,
                                    double horizonSeconds,
                                    const FitState& fitState);
    bool fitSinusoid();
    bool solveLinearModel(const std::vector<const SpeedSample*>& samples,
                          double omega,
                          double timeOrigin,
                          double& sinCoefficient,
                          double& cosCoefficient,
                          double& offset) const;
    void resetFit();
    double normalizeTimestamp(double timestampSeconds);
    [[nodiscard]] bool modelReady() const;

    double defaultPredictionHorizon_ = 0.0;
    int freq_ = 50;
    BigPredictorConfig config_;
    std::deque<SpeedSample> speedSamples_;
    std::optional<FitState> fitState_;
    bool hasPreviousObservation_ = false;
    double previousAngle_ = 0.0;
    double previousTimestamp_ = 0.0;
    double currentTimestamp_ = 0.0;
    double currentAngularVelocity_ = 0.0;
    double syntheticTimestamp_ = 0.0;
    std::size_t acceptedSamplesSinceFit_ = 0;
};

class AngleObserver {
public:
    explicit AngleObserver(ClockMode clockMode);
    double update(double x, double y, double radius);

private:
    double angleTransformer(double x, double y);

    double lastY_ = 0.0;
    double lastX_ = 0.0;
    bool hasLastPosition_ = false;
    std::vector<double> lastAngle_;
    int delta_ = 0;
    ClockMode clockMode_;
};

std::unique_ptr<PredictorInterface> CreatePredictor(MoveMode moveMode,
                                                    double deltaT,
                                                    int freq,
                                                    const BigPredictorConfig& bigConfig = {});

}

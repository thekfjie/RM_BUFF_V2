#pragma once

#include <memory>
#include <optional>
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
    virtual PredictionResult update(double data) = 0;
};

class SmallPredictor final : public PredictorInterface {
public:
    SmallPredictor(double deltaT, int freq);
    PredictionResult update(double data) override;

private:
    int winSize_ = 0;
    double pred_ = 0.0;
    std::vector<double> y_;
};

class BigPredictor final : public PredictorInterface {
public:
    BigPredictor(double deltaT, int freq);
    PredictionResult update(double data) override;

private:
    struct FitState {
        double amplitude = 0.0;
        double omega = 0.0;
        double phase = 0.0;
        double offset = 0.0;
    };

    static double targetValue(double x, const FitState& fitState);
    static FitState fitSinusoid(const std::vector<double>& y);

    int frameInterval_ = 0;
    FitStartDetect startFit_;
    MovAvg smooth_;
    CircularQueue slidWindow_;
    std::optional<FitState> fitState_;
    bool isStart_ = false;
    int fitUpdateCounter_ = 0;
    std::vector<double> y_;
    std::vector<double> diffY_;
    int x_ = 0;
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

std::unique_ptr<PredictorInterface> CreatePredictor(MoveMode moveMode, double deltaT, int freq);

}

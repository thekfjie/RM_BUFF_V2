#include "angle_processor.hpp"

#include <algorithm>
#include <sstream>

#include <opencv2/core/optim.hpp>

namespace gutcpp {

namespace {

struct FitBounds {
    double aMin = 0.0, aMax = 1.0;
    double wMin = 0.0, wMax = 10.0;
    double offsetBase = 0.0;  // offset = offsetBase - amplitude
};

class SinFitObjective final : public cv::DownhillSolver::Function {
public:
    SinFitObjective(std::vector<double> y, FitBounds bounds)
        : y_(std::move(y)), bounds_(bounds) {}

    int getDims() const override {
        return 3;
    }

    double calc(const double* parameters) const override {
        const double amplitude = parameters[0];
        const double omega = parameters[1];
        const double phase = parameters[2];
        const double offset = bounds_.offsetBase - amplitude;
        if (!std::isfinite(amplitude) || !std::isfinite(omega) || !std::isfinite(phase)) {
            return 1e18;
        }

        double penalty = 0.0;
        if (amplitude < bounds_.aMin) penalty += (bounds_.aMin - amplitude) * (bounds_.aMin - amplitude) * 1e6;
        if (amplitude > bounds_.aMax) penalty += (amplitude - bounds_.aMax) * (amplitude - bounds_.aMax) * 1e6;
        if (omega < bounds_.wMin) penalty += (bounds_.wMin - omega) * (bounds_.wMin - omega) * 1e6;
        if (omega > bounds_.wMax) penalty += (omega - bounds_.wMax) * (omega - bounds_.wMax) * 1e6;

        double error = 0.0;
        for (std::size_t index = 0; index < y_.size(); ++index) {
            const double predicted = amplitude * std::sin(omega * static_cast<double>(index) + phase) + offset;
            const double diff = predicted - y_[index];
            error += diff * diff;
        }
        return error + penalty;
    }

private:
    std::vector<double> y_;
    FitBounds bounds_;
};

double EstimateDominantFrequency(const std::vector<double>& y) {
    const int n = static_cast<int>(y.size());
    double bestMagnitude = -1.0;
    double bestFrequency = 1.0 / static_cast<double>(std::max(1, n));
    for (int k = 1; k < n; ++k) {
        double re = 0.0;
        double im = 0.0;
        for (int index = 0; index < n; ++index) {
            const double angle = -2.0 * CV_PI * static_cast<double>(k) * static_cast<double>(index) /
                                 static_cast<double>(n);
            re += y[static_cast<std::size_t>(index)] * std::cos(angle);
            im += y[static_cast<std::size_t>(index)] * std::sin(angle);
        }
        const double magnitude = std::sqrt(re * re + im * im);
        if (magnitude > bestMagnitude) {
            bestMagnitude = magnitude;
            const double frequency = (k <= n / 2) ? static_cast<double>(k) / static_cast<double>(n)
                                                  : static_cast<double>(n - k) / static_cast<double>(n);
            bestFrequency = std::abs(frequency);
        }
    }
    return bestFrequency;
}

}

double EuclideanDistance(const cv::Point2f& p1, const cv::Point2f& p2) {
    const double dx = static_cast<double>(p2.x) - static_cast<double>(p1.x);
    const double dy = static_cast<double>(p2.y) - static_cast<double>(p1.y);
    return std::sqrt(dx * dx + dy * dy);
}

cv::Point2f Rotate(double theta, const cv::Point2f& vector) {
    return {
        static_cast<float>(std::cos(theta) * vector.x + std::sin(theta) * vector.y),
        static_cast<float>(-std::sin(theta) * vector.x + std::cos(theta) * vector.y),
    };
}

double trans(double x, double y) {
    double angle = std::atan2(y, x);
    if (angle < 0.0) {
        angle += CV_PI * 2.0;
    }
    return angle;
}

CircularQueue::CircularQueue(std::size_t queueCapacity)
    : capacity_(queueCapacity), circularQueue_(queueCapacity, -1.0) {
    if (queueCapacity == 0) {
        throw std::runtime_error("Queue capacity must be positive");
    }
}

void CircularQueue::push(double data) {
    if (isFull()) {
        throw std::runtime_error("Queue is full");
    }
    circularQueue_[frontIndex_ % capacity_] = data;
    ++frontIndex_;
}

void CircularQueue::pop() {
    if (isEmpty()) {
        throw std::runtime_error("Queue is empty");
    }
    ++rearIndex_;
}

std::size_t CircularQueue::size() const {
    return frontIndex_ - rearIndex_;
}

bool CircularQueue::isFull() const {
    return size() == capacity_;
}

bool CircularQueue::isEmpty() const {
    return size() == 0;
}

double CircularQueue::front() const {
    if (isEmpty()) {
        throw std::runtime_error("Queue is empty when reading front");
    }
    return circularQueue_[(frontIndex_ - 1) % capacity_];
}

double CircularQueue::rear() const {
    if (isEmpty()) {
        throw std::runtime_error("Queue is empty when reading rear");
    }
    return circularQueue_[rearIndex_ % capacity_];
}

std::size_t CircularQueue::rearIndex() const {
    return rearIndex_;
}

MovAvg::MovAvg(std::size_t windowSize) : preSum_(windowSize), dataQueue_(windowSize) {}

double MovAvg::update(double data) {
    if (firstTime_) {
        preSum_.push(data);
        firstTime_ = false;
    } else {
        if (preSum_.isFull()) {
            preSum_.pop();
            preSum_.push(preSum_.front() - dataQueue_.rear() + data);
            dataQueue_.pop();
        } else {
            preSum_.push(preSum_.front() + data);
        }
    }

    dataQueue_.push(data);
    return preSum_.front() / static_cast<double>(preSum_.size());
}

FitStartDetect::FitStartDetect(std::size_t queueCapacity)
    : queueCapacity_(queueCapacity), queue_(queueCapacity) {}

bool FitStartDetect::isFlip() const {
    return queue_.rearIndex() > 1 && derivative_[idx_ - 2] * derivative_[idx_ - 1] < 0.0;
}

std::pair<bool, int> FitStartDetect::update(double data) {
    queue_.push(data);
    if (queue_.isFull()) {
        queue_.pop();
        derivative_.push_back((queue_.front() - queue_.rear()) / static_cast<double>(queueCapacity_));
        ++idx_;
        if (isFlip()) {
            ++flipCount_;
        }
        if (flipCount_ == 2) {
            if (count_ < lim_) {
                ++count_;
            } else {
                return {true, idx_};
            }
        }
        if (flipCount_ > 2) {
            throw std::runtime_error("Unexpected trend flip count in FitStartDetect");
        }
    }
    return {false, -1};
}

SmallPredictor::SmallPredictor(double deltaT, int freq)
    : deltaAngle_(kSmallRuneSpeed * deltaT),
      warmupFrames_(std::max(1, static_cast<int>(std::ceil(static_cast<double>(freq) * deltaT)))) {}

PredictionResult SmallPredictor::update(double data) {
    ++frameCount_;
    if (frameCount_ == 1) {
        firstAngle_ = data;
    }
    if (frameCount_ >= warmupFrames_) {
        if (direction_ == 0.0) {
            direction_ = (data - firstAngle_ >= 0.0) ? 1.0 : -1.0;
        }
        return {true, direction_ * deltaAngle_};
    }
    return {false, 0.0};
}

BigPredictor::BigPredictor(double deltaT, int freq)
    : frameInterval_(static_cast<int>(std::ceil(static_cast<double>(freq) * deltaT))),
      freq_(freq),
      startFit_(),
      smooth_(20),
      slidWindow_(static_cast<std::size_t>(frameInterval_)) {}

double BigPredictor::targetValue(double x, const FitState& fitState) {
    return fitState.amplitude * std::sin(fitState.omega * x + fitState.phase) + fitState.offset;
}

BigPredictor::FitState BigPredictor::fitSinusoid(const std::vector<double>& y) const {
    if (y.size() < 4) {
        throw std::runtime_error("Not enough samples to fit big predictor");
    }

    // Physical rule: spd = a*sin(w*t) + b, a∈[0.780,1.045], w∈[1.884,2.000], b=2.090-a
    // Fitting data diffY[i] ≈ speed * (frameInterval/freq), index i is per-frame
    // So: a_fit = a_phys * timeScale, w_fit = w_phys / freq, offset_fit = b_phys * timeScale
    const double timeScale = static_cast<double>(frameInterval_) / static_cast<double>(freq_);
    const double freqScale = 1.0 / static_cast<double>(freq_);

    const FitBounds bounds{
        0.780 * timeScale,   // aMin
        1.045 * timeScale,   // aMax
        1.884 * freqScale,   // wMin
        2.000 * freqScale,   // wMax
        2.090 * timeScale    // offsetBase: offset = offsetBase - amplitude
    };

    auto evaluateError = [&](const FitState& state) {
        double error = 0.0;
        for (std::size_t index = 0; index < y.size(); ++index) {
            const double predicted = state.amplitude * std::sin(state.omega * static_cast<double>(index) + state.phase) +
                                     state.offset;
            const double diff = predicted - y[index];
            error += diff * diff;
        }
        return error;
    };

    FitState fitState;
    const auto [minIt, maxIt] = std::minmax_element(y.begin(), y.end());
    fitState.amplitude = std::clamp(*maxIt - *minIt, bounds.aMin, bounds.aMax);
    fitState.omega = std::clamp(2.0 * CV_PI * EstimateDominantFrequency(y), bounds.wMin, bounds.wMax);
    fitState.phase = 0.0;
    fitState.offset = bounds.offsetBase - fitState.amplitude;

    cv::Mat params = (cv::Mat_<double>(3, 1) << fitState.amplitude, fitState.omega, fitState.phase);
    cv::Mat step = (cv::Mat_<double>(3, 1) << (bounds.aMax - bounds.aMin) * 0.3,
                   (bounds.wMax - bounds.wMin) * 0.5, 0.5);

    const FitState initialState = fitState;
    const double initialError = evaluateError(initialState);

    cv::Ptr<cv::DownhillSolver> solver = cv::DownhillSolver::create();
    solver->setFunction(cv::makePtr<SinFitObjective>(y, bounds));
    solver->setInitStep(step);
    solver->setTermCriteria(cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 5000, 1e-9));
    solver->minimize(params);

    FitState optimizedState;
    optimizedState.amplitude = std::clamp(params.at<double>(0, 0), bounds.aMin, bounds.aMax);
    optimizedState.omega = std::clamp(params.at<double>(1, 0), bounds.wMin, bounds.wMax);
    optimizedState.phase = params.at<double>(2, 0);
    optimizedState.offset = bounds.offsetBase - optimizedState.amplitude;

    const double optimizedError = evaluateError(optimizedState);
    if (std::isfinite(optimizedError) && optimizedError <= initialError) {
        fitState = optimizedState;
    }
    return fitState;
}

PredictionResult BigPredictor::update(double data) {
    y_.push_back(data);
    ++x_;
    slidWindow_.push(data);
    if (slidWindow_.isFull()) {
        double diff = slidWindow_.front() - slidWindow_.rear();
        diff = smooth_.update(diff);
        diffY_.push_back(diff);
        slidWindow_.pop();
        if (!isStart_) {
            const auto [flag, _] = startFit_.update(diff);
            (void) _;
            isStart_ = flag;
        }
        if (isStart_) {
            if (!fitState_.has_value() || fitUpdateCounter_ % frameInterval_ == 0) {
                fitState_ = fitSinusoid(diffY_);
            }
            ++fitUpdateCounter_;
            return {true, targetValue(static_cast<double>(x_ + frameInterval_), fitState_.value())};
        }
    }
    return {false, 0.0};
}

AngleObserver::AngleObserver(ClockMode clockMode) : clockMode_(clockMode) {}

double AngleObserver::angleTransformer(double x, double y) {
    double theta = std::atan2(y, x);
    if (lastAngle_.empty()) {
        lastAngle_.push_back(theta);
        return theta;
    }

    double delta = std::fabs(std::round((theta - lastAngle_.front()) / CV_PI)) * CV_PI;
    if (clockMode_ == ClockMode::Anticlockwise) {
        delta *= -1.0;
    }
    theta += delta;
    lastAngle_.front() = theta;
    return theta;
}

double AngleObserver::update(double x, double y, double radius) {
    if (!hasLastPosition_) {
        lastX_ = x;
        lastY_ = y;
        hasLastPosition_ = true;
    }
    if (delta_ != 0) {
        const cv::Point2f rotated = Rotate(2.0 * CV_PI / 5.0 * static_cast<double>(5 - delta_),
                                           cv::Point2f(static_cast<float>(x), static_cast<float>(y)));
        x = rotated.x;
        y = rotated.y;
    }
    if (EuclideanDistance(cv::Point2f(static_cast<float>(lastX_), static_cast<float>(lastY_)),
                          cv::Point2f(static_cast<float>(x), static_cast<float>(y))) > radius * 0.5) {
        std::vector<cv::Point2f> points;
        points.reserve(5);
        for (int time = 0; time < 5; ++time) {
            points.push_back(Rotate(2.0 * CV_PI / 5.0 * static_cast<double>(time),
                                    cv::Point2f(static_cast<float>(lastX_), static_cast<float>(lastY_))));
        }
        int bestIndex = 0;
        double bestDistance = std::numeric_limits<double>::max();
        for (int index = 0; index < 5; ++index) {
            const double distance = EuclideanDistance(cv::Point2f(static_cast<float>(x), static_cast<float>(y)), points[index]);
            if (distance < bestDistance) {
                bestDistance = distance;
                bestIndex = index;
            }
        }
        const cv::Point2f rotated = Rotate(2.0 * CV_PI / 5.0 * static_cast<double>(5 - bestIndex),
                                           cv::Point2f(static_cast<float>(x), static_cast<float>(y)));
        x = rotated.x;
        y = rotated.y;
        delta_ += bestIndex;
    }
    const double angle = angleTransformer(x, y);
    lastX_ = x;
    lastY_ = y;
    return angle;
}

std::string SmallPredictor::debugState() const {
    std::ostringstream oss;
    oss << "Small,deltaAngle=" << direction_ * deltaAngle_ << ",dir=" << direction_
        << ",frame=" << frameCount_ << ",warmup=" << warmupFrames_;
    return oss.str();
}

std::string BigPredictor::debugState() const {
    std::ostringstream oss;
    oss << "Big,isStart=" << isStart_ << ",refitCount=" << fitUpdateCounter_;
    if (fitState_.has_value()) {
        const auto& fs = fitState_.value();
        const double timeScale = static_cast<double>(frameInterval_) / static_cast<double>(freq_);
        const double freqScale = 1.0 / static_cast<double>(freq_);
        oss << ",a_fit=" << fs.amplitude << ",w_fit=" << fs.omega
            << ",phase=" << fs.phase << ",offset=" << fs.offset
            << ",a_phys=" << fs.amplitude / timeScale
            << ",w_phys=" << fs.omega / freqScale;
    } else {
        oss << ",fit=none";
    }
    return oss.str();
}

std::unique_ptr<PredictorInterface> CreatePredictor(MoveMode moveMode, double deltaT, int freq) {
    if (moveMode == MoveMode::Small) {
        return std::make_unique<SmallPredictor>(deltaT, freq);
    }
    return std::make_unique<BigPredictor>(deltaT, freq);
}

}

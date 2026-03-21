#include "angle_processor.hpp"

#include <algorithm>
#include <numeric>

#include <opencv2/core/optim.hpp>

namespace gutcpp {

namespace {

class SinFitObjective final : public cv::DownhillSolver::Function {
public:
    explicit SinFitObjective(std::vector<double> y) : y_(std::move(y)) {}

    int getDims() const override {
        return 4;
    }

    double calc(const double* parameters) const override {
        const double amplitude = parameters[0];
        const double omega = parameters[1];
        const double phase = parameters[2];
        const double offset = parameters[3];
        if (!std::isfinite(amplitude) || !std::isfinite(omega) || !std::isfinite(phase) || !std::isfinite(offset) ||
            omega <= 0.0) {
            return 1e18;
        }

        double error = 0.0;
        for (std::size_t index = 0; index < y_.size(); ++index) {
            const double predicted = amplitude * std::sin(omega * static_cast<double>(index) + phase) + offset;
            const double diff = predicted - y_[index];
            error += diff * diff;
        }
        return error;
    }

private:
    std::vector<double> y_;
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
    : winSize_(static_cast<int>(std::ceil(static_cast<double>(freq) * deltaT))) {}

PredictionResult SmallPredictor::update(double data) {
    y_.push_back(data);
    if (static_cast<int>(y_.size()) == winSize_) {
        std::vector<double> diff;
        diff.reserve(static_cast<std::size_t>(std::max(0, winSize_ - 1)));
        for (int index = 1; index < winSize_; ++index) {
            diff.push_back(y_[static_cast<std::size_t>(index)] - y_[static_cast<std::size_t>(index - 1)]);
        }
        pred_ = std::accumulate(diff.begin(), diff.end(), 0.0) / static_cast<double>(diff.size());
        return {true, pred_ * static_cast<double>(winSize_)};
    }
    if (static_cast<int>(y_.size()) > winSize_) {
        return {true, pred_ * static_cast<double>(winSize_)};
    }
    return {false, 0.0};
}

BigPredictor::BigPredictor(double deltaT, int freq)
    : frameInterval_(static_cast<int>(std::ceil(static_cast<double>(freq) * deltaT))),
      startFit_(),
      smooth_(20),
      slidWindow_(static_cast<std::size_t>(frameInterval_)) {}

double BigPredictor::targetValue(double x, const FitState& fitState) {
    return fitState.amplitude * std::sin(fitState.omega * x + fitState.phase) + fitState.offset;
}

BigPredictor::FitState BigPredictor::fitSinusoid(const std::vector<double>& y) {
    if (y.size() < 4) {
        throw std::runtime_error("Not enough samples to fit big predictor");
    }

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
    fitState.amplitude = *maxIt - *minIt;
    fitState.omega = 2.0 * CV_PI * EstimateDominantFrequency(y);
    fitState.phase = 0.0;
    fitState.offset = std::accumulate(y.begin(), y.end(), 0.0) / static_cast<double>(y.size());

    cv::Mat params = (cv::Mat_<double>(4, 1) << fitState.amplitude, std::max(1e-6, fitState.omega), fitState.phase,
                     fitState.offset);
    cv::Mat step = (cv::Mat_<double>(4, 1) << std::max(0.1, std::abs(fitState.amplitude) * 0.1),
                   std::max(1e-4, std::abs(fitState.omega) * 0.1), 0.5,
                   std::max(0.1, std::abs(fitState.offset) * 0.1 + 0.1));

    const FitState initialState = fitState;
    const double initialError = evaluateError(initialState);

    cv::Ptr<cv::DownhillSolver> solver = cv::DownhillSolver::create();
    solver->setFunction(cv::makePtr<SinFitObjective>(y));
    solver->setInitStep(step);
    solver->setTermCriteria(cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 5000, 1e-9));
    solver->minimize(params);

    FitState optimizedState;
    optimizedState.amplitude = params.at<double>(0, 0);
    optimizedState.omega = params.at<double>(1, 0);
    optimizedState.phase = params.at<double>(2, 0);
    optimizedState.offset = params.at<double>(3, 0);

    const double optimizedError = evaluateError(optimizedState);
    if (std::isfinite(optimizedError) && optimizedState.omega > 0.0 && optimizedError <= initialError) {
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

std::unique_ptr<PredictorInterface> CreatePredictor(MoveMode moveMode, double deltaT, int freq) {
    if (moveMode == MoveMode::Small) {
        return std::make_unique<SmallPredictor>(deltaT, freq);
    }
    return std::make_unique<BigPredictor>(deltaT, freq);
}

}

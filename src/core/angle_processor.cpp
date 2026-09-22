#include "angle_processor.hpp"

#include <algorithm>
#include <cmath>
#include <random>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace gutcpp {

namespace {

constexpr double kMinimumObservationDt = 1e-5;

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
      defaultPredictionHorizon_(std::max(0.0, deltaT)),
      warmupFrames_(std::max(5, static_cast<int>(std::ceil(static_cast<double>(freq) * std::max(0.1, deltaT))))),
      freq_(std::max(1, freq)) {}

PredictionResult SmallPredictor::update(double data, double timestampSeconds) {
    if (!std::isfinite(data)) {
        hasPrevious_ = false;
        direction_ = 0.0;
        return {};
    }
    const double timestamp = std::isfinite(timestampSeconds) ? timestampSeconds :
        (hasPrevious_ ? previousTimestamp_ + 1.0 / freq_ : 0.0);
    const double dt = timestamp - previousTimestamp_;
    const double change = data - previousAngle_;
    if (!hasPrevious_ || dt <= kMinimumObservationDt || dt > 0.5 ||
        std::abs(change) > 3.0 * dt + 0.02) {
        hasPrevious_ = true;
        frameCount_ = 1;
        directionEvidence_ = 0;
        direction_ = 0.0;
        filteredPhase_ = data;
        phaseVariance_ = 0.0004;
        previousTimestamp_ = timestamp;
        previousAngle_ = data;
        return {};
    }
    previousTimestamp_ = timestamp;
    previousAngle_ = data;
    ++frameCount_;
    if (std::abs(change) > 1e-4) {
        directionEvidence_ = std::clamp(directionEvidence_ + (change > 0.0 ? 1 : -1), -8, 8);
    }
    const double confirmedDirection = directionEvidence_ >= 5 ? 1.0 :
                                      directionEvidence_ <= -5 ? -1.0 : direction_;
    const bool directionChanged = confirmedDirection != direction_;
    if (directionChanged) {
        direction_ = confirmedDirection;
        filteredPhase_ = data;
        phaseVariance_ = 0.0004;
    }
    if (direction_ == 0.0 || frameCount_ < warmupFrames_) {
        filteredPhase_ = data;
        return {};
    }
    // Scalar phase KF with known signed angular speed. Time is measured, not
    // inferred from frame count. The posterior is used by the pixel predictor.
    const double prior = directionChanged ? data : filteredPhase_ + direction_ * kSmallRuneSpeed * dt;
    phaseVariance_ += 0.005 * dt;
    const double gain = phaseVariance_ / (phaseVariance_ + 0.0004);
    filteredPhase_ = prior + gain * (data - prior);
    phaseVariance_ *= 1.0 - gain;
    return {true, predictDelta(defaultPredictionHorizon_), direction_ * kSmallRuneSpeed,
            true, filteredPhase_ - data};
}

double SmallPredictor::predictDelta(double horizonSeconds) const {
    return direction_ * kSmallRuneSpeed * std::max(0.0, horizonSeconds);
}

BigPredictor::BigPredictor(double deltaT, int freq, BigPredictorConfig config)
    : defaultPredictionHorizon_(std::max(0.0, deltaT)),
      freq_(std::max(1, freq)),
      config_(std::move(config)) {
    config_.omegaSearchSteps = std::max(2, config_.omegaSearchSteps);
    config_.fitUpdateStride = std::max(1, config_.fitUpdateStride);
    config_.minInliers = std::max<std::size_t>(3, config_.minInliers);
    config_.maxSamples = std::max(config_.minInliers, config_.maxSamples);
    config_.minInlierRatio = std::clamp(config_.minInlierRatio, 0.0, 1.0);
    config_.inlierThreshold = std::max(1e-6, config_.inlierThreshold);
    if (config_.minOmega > config_.maxOmega) {
        std::swap(config_.minOmega, config_.maxOmega);
    }
    if (config_.minAmplitude > config_.maxAmplitude) {
        std::swap(config_.minAmplitude, config_.maxAmplitude);
    }
    config_.minAmplitude = std::max(0.0, config_.minAmplitude);
    config_.maxAmplitude = std::max(config_.minAmplitude, config_.maxAmplitude);
    config_.maxAbsSpeed = std::max(1e-6, config_.maxAbsSpeed);
    config_.maxObservationGap = std::max(kMinimumObservationDt, config_.maxObservationGap);
    config_.maxPhaseJump = std::max(1e-6, config_.maxPhaseJump);
    config_.minSampleSpan = std::max(0.0, config_.minSampleSpan);
    config_.maxModelAge = std::max(kMinimumObservationDt, config_.maxModelAge);
    config_.maxConsecutiveRejected = std::max(1, config_.maxConsecutiveRejected);
}

double BigPredictor::velocityAt(double timestamp, const FitState& fitState) {
    const double relativeTime = timestamp - fitState.timeOrigin;
    return fitState.amplitude * std::sin(fitState.omega * relativeTime + fitState.phase) +
           fitState.offset;
}

double BigPredictor::integrateVelocity(double startTimestamp,
                                       double horizonSeconds,
                                       const FitState& fitState) {
    const double horizon = std::max(0.0, horizonSeconds);
    if (horizon == 0.0) {
        return 0.0;
    }

    const double phaseAtStart =
        fitState.omega * (startTimestamp - fitState.timeOrigin) + fitState.phase;
    if (std::abs(fitState.omega) <= 1e-9) {
        return (fitState.amplitude * std::sin(phaseAtStart) + fitState.offset) * horizon;
    }

    return fitState.offset * horizon +
           fitState.amplitude *
               (std::cos(phaseAtStart) -
                std::cos(phaseAtStart + fitState.omega * horizon)) /
               fitState.omega;
}

bool BigPredictor::solveLinearModel(const std::vector<const SpeedSample*>& samples,
                                    double omega,
                                    double timeOrigin,
                                    double& sinCoefficient,
                                    double& cosCoefficient,
                                    double& offset) const {
    if (samples.size() < 3 || !std::isfinite(omega)) {
        return false;
    }

    cv::Mat design(static_cast<int>(samples.size()), 3, CV_64F);
    cv::Mat observations(static_cast<int>(samples.size()), 1, CV_64F);
    for (std::size_t index = 0; index < samples.size(); ++index) {
        const double relativeTime = samples[index]->timestamp - timeOrigin;
        design.at<double>(static_cast<int>(index), 0) = std::sin(omega * relativeTime);
        design.at<double>(static_cast<int>(index), 1) = std::cos(omega * relativeTime);
        design.at<double>(static_cast<int>(index), 2) = 1.0;
        observations.at<double>(static_cast<int>(index), 0) = samples[index]->velocity;
    }

    // Three points with nearly identical phases cannot identify the model.
    cv::SVD svd(design, cv::SVD::NO_UV);
    if (svd.w.total() < 3 || svd.w.at<double>(2) <= 1e-6 * svd.w.at<double>(0)) return false;
    cv::Mat parameters;
    if (!cv::solve(design, observations, parameters, cv::DECOMP_SVD) ||
        parameters.rows != 3 || parameters.cols != 1) {
        return false;
    }

    sinCoefficient = parameters.at<double>(0, 0);
    cosCoefficient = parameters.at<double>(1, 0);
    offset = parameters.at<double>(2, 0);
    return std::isfinite(sinCoefficient) && std::isfinite(cosCoefficient) &&
           std::isfinite(offset);
}

bool BigPredictor::fitSinusoid() {
    if (speedSamples_.size() < 3 ||
        speedSamples_.back().timestamp - speedSamples_.front().timestamp < config_.minSampleSpan) {
        fitState_.reset();
        return false;
    }

    std::vector<const SpeedSample*> allSamples;
    allSamples.reserve(speedSamples_.size());
    for (const SpeedSample& sample : speedSamples_) {
        allSamples.push_back(&sample);
    }

    const double timeOrigin = speedSamples_.front().timestamp;
    std::optional<FitState> bestFit;
    // Repeatable RANSAC: one random three-point hypothesis per omega grid
    // location, plus the full-window hypothesis for clean data. Refit consensus.
    std::mt19937 random(0x42554646u);
    const std::size_t third = allSamples.size() / 3;
    for (int hypothesis = 0; hypothesis < 2 * config_.omegaSearchSteps; ++hypothesis) {
        const int step = hypothesis / 2;
        const double ratio = static_cast<double>(step) /
                             static_cast<double>(config_.omegaSearchSteps - 1);
        const double omega = config_.minOmega +
                             (config_.maxOmega - config_.minOmega) * ratio;

        double sinCoefficient = 0.0;
        double cosCoefficient = 0.0;
        double offset = 0.0;
        std::vector<const SpeedSample*> subset;
        if (hypothesis % 2 != 0) {
            // Stratification spreads samples in time and avoids duplicate IDs.
            for (std::size_t part = 0; part < 3; ++part) {
                const std::size_t begin = part * third;
                const std::size_t end = part == 2 ? allSamples.size() : (part + 1) * third;
                std::uniform_int_distribution<std::size_t> choose(begin, end - 1);
                subset.push_back(allSamples[choose(random)]);
            }
        }
        const auto& seedSamples = subset.empty() ? allSamples : subset;
        if (!solveLinearModel(seedSamples,
                              omega,
                              timeOrigin,
                              sinCoefficient,
                              cosCoefficient,
                              offset)) {
            continue;
        }

        std::vector<const SpeedSample*> inliers;
        inliers.reserve(allSamples.size());
        for (const SpeedSample* sample : allSamples) {
            const double relativeTime = sample->timestamp - timeOrigin;
            const double predicted =
                sinCoefficient * std::sin(omega * relativeTime) +
                cosCoefficient * std::cos(omega * relativeTime) + offset;
            if (std::abs(sample->velocity - predicted) < config_.inlierThreshold) {
                inliers.push_back(sample);
            }
        }
        if (inliers.size() < 3) {
            continue;
        }

        if (!solveLinearModel(inliers,
                              omega,
                              timeOrigin,
                              sinCoefficient,
                              cosCoefficient,
                              offset)) {
            continue;
        }

        const double amplitude = std::hypot(sinCoefficient, cosCoefficient);
        if (!std::isfinite(amplitude) ||
            amplitude < config_.minAmplitude || amplitude > config_.maxAmplitude ||
            std::abs(offset) + amplitude >
                config_.maxAbsSpeed + 2.0 * config_.inlierThreshold) {
            continue;
        }

        std::size_t finalInliers = 0;
        double squaredError = 0.0;
        for (const SpeedSample* sample : allSamples) {
            const double relativeTime = sample->timestamp - timeOrigin;
            const double predicted =
                sinCoefficient * std::sin(omega * relativeTime) +
                cosCoefficient * std::cos(omega * relativeTime) + offset;
            const double residual = sample->velocity - predicted;
            if (std::abs(residual) < config_.inlierThreshold) {
                ++finalInliers;
                squaredError += residual * residual;
            }
        }
        if (finalInliers < 3) {
            continue;
        }

        FitState candidate;
        candidate.amplitude = amplitude;
        candidate.omega = omega;
        candidate.phase = std::atan2(cosCoefficient, sinCoefficient);
        candidate.offset = offset;
        candidate.timeOrigin = timeOrigin;
        candidate.inliers = finalInliers;
        candidate.rmse = std::sqrt(squaredError / static_cast<double>(finalInliers));

        if (!bestFit.has_value() ||
            candidate.inliers > bestFit->inliers ||
            (candidate.inliers == bestFit->inliers && candidate.rmse < bestFit->rmse)) {
            bestFit = candidate;
        }
    }

    fitState_ = bestFit;
    return modelReady();
}

void BigPredictor::resetFit() {
    speedSamples_.clear();
    fitState_.reset();
    currentAngularVelocity_ = 0.0;
    acceptedSamplesSinceFit_ = 0;
    lastAcceptedTimestamp_ = -std::numeric_limits<double>::infinity();
    consecutiveRejected_ = 0;
}

double BigPredictor::normalizeTimestamp(double timestampSeconds) {
    if (std::isfinite(timestampSeconds)) {
        syntheticTimestamp_ = timestampSeconds;
        return timestampSeconds;
    }

    if (hasPreviousObservation_) {
        syntheticTimestamp_ += 1.0 / static_cast<double>(freq_);
    }
    return syntheticTimestamp_;
}

bool BigPredictor::modelReady() const {
    if (!fitState_.has_value() || speedSamples_.empty() ||
        currentTimestamp_ - lastAcceptedTimestamp_ > config_.maxModelAge ||
        consecutiveRejected_ >= config_.maxConsecutiveRejected) {
        return false;
    }
    const double inlierRatio = static_cast<double>(fitState_->inliers) /
                               static_cast<double>(speedSamples_.size());
    return fitState_->inliers >= config_.minInliers &&
           inlierRatio >= config_.minInlierRatio &&
           std::isfinite(fitState_->rmse);
}

PredictionResult BigPredictor::update(double data, double timestampSeconds) {
    const double timestamp = normalizeTimestamp(timestampSeconds);
    currentTimestamp_ = timestamp;
    if (!std::isfinite(data) || !std::isfinite(timestamp)) {
        resetFit();
        hasPreviousObservation_ = false;
        return {};
    }

    if (!hasPreviousObservation_) {
        hasPreviousObservation_ = true;
        previousAngle_ = data;
        previousTimestamp_ = timestamp;
        return {};
    }

    const double dt = timestamp - previousTimestamp_;
    if (dt <= kMinimumObservationDt || dt > config_.maxObservationGap) {
        resetFit();
        previousAngle_ = data;
        previousTimestamp_ = timestamp;
        return {};
    }

    const double phaseDelta = data - previousAngle_;
    previousAngle_ = data;
    previousTimestamp_ = timestamp;
    if (!std::isfinite(phaseDelta) || std::abs(phaseDelta) > config_.maxPhaseJump) {
        resetFit();
        return {};
    }

    const double velocity = phaseDelta / dt;
    if (!std::isfinite(velocity) || std::abs(velocity) > config_.maxAbsSpeed) {
        ++consecutiveRejected_;
        if (consecutiveRejected_ >= config_.maxConsecutiveRejected ||
            timestamp - lastAcceptedTimestamp_ > config_.maxModelAge) resetFit();
        // A rejected current measurement is never reported as a ready target.
        return {};
    }

    const double midpoint = timestamp - dt * 0.5;
    const bool agreesWithModel = !fitState_.has_value() ||
        std::abs(velocity - velocityAt(midpoint, *fitState_)) <= config_.inlierThreshold;
    consecutiveRejected_ = agreesWithModel ? 0 : consecutiveRejected_ + 1;
    if (consecutiveRejected_ >= config_.maxConsecutiveRejected) resetFit();
    lastAcceptedTimestamp_ = timestamp;
    currentAngularVelocity_ = velocity;
    speedSamples_.push_back({timestamp - dt * 0.5, velocity});
    while (speedSamples_.size() > config_.maxSamples) {
        speedSamples_.pop_front();
    }

    ++acceptedSamplesSinceFit_;
    if (speedSamples_.size() >= config_.minInliers &&
        (!fitState_.has_value() ||
         acceptedSamplesSinceFit_ >= static_cast<std::size_t>(config_.fitUpdateStride))) {
        fitSinusoid();
        acceptedSamplesSinceFit_ = 0;
    }

    const bool ready = modelReady() && agreesWithModel;
    if (ready) {
        currentAngularVelocity_ = velocityAt(timestamp, fitState_.value());
    }
    return {ready,
            ready ? predictDelta(defaultPredictionHorizon_) : 0.0,
            currentAngularVelocity_,
            ready};
}

double BigPredictor::predictDelta(double horizonSeconds) const {
    if (!modelReady()) {
        return 0.0;
    }
    return integrateVelocity(currentTimestamp_, horizonSeconds, fitState_.value());
}

AngleObserver::AngleObserver(ClockMode /*clockMode*/) {}

double AngleObserver::update(double x, double y, double radius) {
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(radius) ||
        radius <= 0.0 || std::hypot(x, y) < 1e-6) {
        hasPrevious_ = false;
        return std::numeric_limits<double>::quiet_NaN();
    }
    const double raw = std::atan2(y, x);
    if (!hasPrevious_) {
        previousRawAngle_ = continuousAngle_ = raw;
        hasPrevious_ = true;
        return continuousAngle_;
    }
    double delta = std::remainder(raw - previousRawAngle_, 2.0 * CV_PI);
    // Separate a change of blade from continuous rotor phase. This assumes
    // less than half a blade spacing of motion between valid observations.
    // The pipeline resets history across long observation gaps.
    if (std::abs(delta) > CV_PI / 5.0) {
        delta = std::remainder(delta, 2.0 * CV_PI / 5.0);
    }
    continuousAngle_ += delta;
    previousRawAngle_ = raw;
    return continuousAngle_;
}

std::string SmallPredictor::debugState() const {
    std::ostringstream oss;
    oss << "Small,deltaAngle=" << direction_ * deltaAngle_ << ",dir=" << direction_
        << ",frame=" << frameCount_ << ",warmup=" << warmupFrames_;
    return oss.str();
}

std::string BigPredictor::debugState() const {
    std::ostringstream oss;
    oss << "Big,ready=" << modelReady()
        << ",samples=" << speedSamples_.size()
        << ",v=" << currentAngularVelocity_;
    if (fitState_.has_value()) {
        const auto& fs = fitState_.value();
        oss << ",inliers=" << fs.inliers
            << ",rmse=" << fs.rmse
            << ",a=" << fs.amplitude
            << ",w=" << fs.omega
            << ",phase=" << fs.phase
            << ",offset=" << fs.offset;
    } else {
        oss << ",fit=none";
    }
    return oss.str();
}

std::unique_ptr<PredictorInterface> CreatePredictor(MoveMode moveMode,
                                                    double deltaT,
                                                    int freq,
                                                    const BigPredictorConfig& bigConfig) {
    if (moveMode == MoveMode::Small) {
        return std::make_unique<SmallPredictor>(deltaT, freq);
    }
    return std::make_unique<BigPredictor>(deltaT, freq, bigConfig);
}

}

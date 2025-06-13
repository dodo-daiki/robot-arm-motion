#include "Joint.h"

Joint::Joint(float theta_offset, float limit_min, float limit_max)
    : theta_offset_(theta_offset),
      limit_min_(limit_min),
      limit_max_(limit_max),
      current_value_(0.0f) {}

void Joint::setCurrentValue(float value) {
    current_value_ = clamp(value, limit_min_, limit_max_);
}

float Joint::getCurrentValue() const {
    return current_value_;
}

float Joint::getThetaOffset() const {
    return theta_offset_;
}

float Joint::getLimitMin() const {
    return limit_min_;
}

float Joint::getLimitMax() const {
    return limit_max_;
}

float Joint::getClampedValue(float value) const {
    return clamp(value, limit_min_, limit_max_);
}

float Joint::clamp(float val, float min_val, float max_val) const {
    if (val < min_val) return min_val;
    if (val > max_val) return max_val;
    return val;
}

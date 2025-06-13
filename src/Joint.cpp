#include "Joint.h"

Joint::Joint(double minLimit_, double maxLimit_, const Eigen::Vector3d& axis_)
    : angle(0.0), minLimit(minLimit_), maxLimit(maxLimit_), axis(axis_) {}

double Joint::getAngle() const {
    return angle;
}

void Joint::setAngle(double angle_) {
    angle = angle_;
}

double Joint::getMinLimit() const {
    return minLimit;
}

double Joint::getMaxLimit() const {
    return maxLimit;
}

Eigen::Vector3d Joint::getAxis() const {
    return axis;
}

void Joint::setAxis(const Eigen::Vector3d& axis_) {
    axis = axis_;
}

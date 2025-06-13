#define _USE_MATH_DEFINES
#include <cmath>

#include "MathUtils.h"
#include <cmath>

Eigen::Vector3d MathUtils::rotToAA(const Eigen::Matrix3d& R) {
    Eigen::AngleAxisd aa(R);
    return aa.angle() * aa.axis();
}

Eigen::Matrix3d MathUtils::aaToRot(const Eigen::Vector3d& vec) {
    double angle = vec.norm();
    if (angle < 1e-8) {
        return Eigen::Matrix3d::Identity();
    }
    Eigen::Vector3d axis = vec / angle;
    return Eigen::AngleAxisd(angle, axis).toRotationMatrix();
}

double MathUtils::clamp(double value, double minVal, double maxVal) {
    if (value < minVal) return minVal;
    if (value > maxVal) return maxVal;
    return value;
}

double MathUtils::normAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

Eigen::MatrixXd MathUtils::dampingMat(size_t size, double lambda) {
    return lambda * Eigen::MatrixXd::Identity(size, size);
}

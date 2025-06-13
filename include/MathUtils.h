#pragma once
#include <Eigen/Dense>

class MathUtils {
public:
    static Eigen::Vector3d rotToAA(const Eigen::Matrix3d& R);

    static Eigen::Matrix3d aaToRot(const Eigen::Vector3d& vec);

    static double clamp(double value, double minVal, double maxVal);

    static double normAngle(double angle);

    static Eigen::MatrixXd dampingMat(size_t size, double lambda);
};

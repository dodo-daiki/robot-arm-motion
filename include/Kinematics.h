#pragma once
#include "Body.h"
#include "MathUtils.h"
#include <Eigen/Dense>
#include <vector>

class Kinematics {
public:
    Kinematics(const Body& body_);

    Eigen::Affine3d forwardKinematics(const std::vector<double>& jointAngles) const;
    Eigen::MatrixXd computeJacobian(const std::vector<double>& jointAngles) const;
    bool inverseKinematics(const Eigen::Affine3d& targetPose, std::vector<double>& jointAngles) const;

private:
    const Body& body;
};

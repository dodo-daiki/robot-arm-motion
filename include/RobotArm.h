#pragma once
#include "Body.h"
#include "Kinematics.h"
#include <Eigen/Dense>
#include <vector>

class RobotArm {
public:
    RobotArm(const Body& body_);

    Eigen::Affine3d forwardKinematics(const std::vector<double>& jointAngles) const;
    Eigen::MatrixXd computeJacobian(const std::vector<double>& jointAngles) const;
    bool inverseKinematics(const Eigen::Affine3d& targetPose, std::vector<double>& jointAngles);

private:
    const Body& body;
    Kinematics kinematics;
};

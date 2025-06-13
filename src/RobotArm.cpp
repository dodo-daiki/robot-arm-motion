#include "RobotArm.h"

RobotArm::RobotArm(const Body& body_) : body(body_), kinematics(body_) {}

Eigen::Affine3d RobotArm::forwardKinematics(const std::vector<double>& jointAngles) const {
    return kinematics.forwardKinematics(jointAngles);
}

Eigen::MatrixXd RobotArm::computeJacobian(const std::vector<double>& jointAngles) const {
    return kinematics.computeJacobian(jointAngles);
}

bool RobotArm::inverseKinematics(const Eigen::Affine3d& targetPose, std::vector<double>& jointAngles) {
    return kinematics.inverseKinematics(targetPose, jointAngles);
}

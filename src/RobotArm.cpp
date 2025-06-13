#include "RobotArm.h"

RobotArm::RobotArm()
    : solver_(joints_, links_)  // KinematicsSolverに参照を渡す（joints_とlinks_を共有）
{}

void RobotArm::addJoint(float limit_min, float limit_max, float theta_offset) {
    joints_.emplace_back(theta_offset, limit_min, limit_max);
}

void RobotArm::addLink(float a, float alpha, float d) {
    links_.emplace_back(a, alpha, d);
}

size_t RobotArm::getNumJoints() const {
    return joints_.size();
}

void RobotArm::computeIK(float target_x, float target_y, float target_z,
                         const std::vector<float>& initial_guess) {
    solver_.computeIK(target_x, target_y, target_z, initial_guess);
}

float RobotArm::getJointAngle(size_t index) const {
    if (index >= joints_.size()) return 0.0f;  // 安全のため
    return joints_[index].getCurrentValue();
}

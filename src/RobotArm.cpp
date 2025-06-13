#include "RobotArm.h"

RobotArm::RobotArm()
    : solver_(joints_, links_) {}

void RobotArm::addJoint(float limit_min, float limit_max, float theta_offset) {
    joints_.emplace_back(theta_offset, limit_min, limit_max);
}

void RobotArm::addLink(float a, float alpha, float d) {
    links_.emplace_back(a, alpha, d);
}

size_t RobotArm::getNumJoints() const {
    return joints_.size();
}

// IKパラメータ設定のパススルー
void RobotArm::setMaxIterations(int max_iterations) {
    solver_.setMaxIterations(max_iterations);
}

void RobotArm::setTolerance(float tolerance) {
    solver_.setTolerance(tolerance);
}

void RobotArm::setLearningRate(float learning_rate) {
    solver_.setLearningRate(learning_rate);
}

void RobotArm::computeIK(float target_x, float target_y, float target_z,
                         float target_roll, float target_pitch, float target_yaw) {

    std::vector<float> initial_guess(joints_.size());
    for (size_t i = 0; i < joints_.size(); ++i) {
        initial_guess[i] = joints_[i].getCurrentValue();
    }

    solver_.computeIK(target_x, target_y, target_z,
                      target_roll, target_pitch, target_yaw,
                      initial_guess);
}

void RobotArm::getEndEffectorPose(float& x, float& y, float& z,
                                  float& roll, float& pitch, float& yaw) {
    std::vector<float> joint_angles(joints_.size());
    for (size_t i = 0; i < joints_.size(); ++i) {
        joint_angles[i] = joints_[i].getCurrentValue();
    }
    solver_.forwardKinematics(joint_angles, x, y, z, roll, pitch, yaw);
}


void RobotArm::setJointAngle(size_t index, float angle) {
    if (index >= joints_.size()) return;
    
    joints_[index].setCurrentValue(angle);
}


float RobotArm::getJointAngle(size_t index) const {
    if (index >= joints_.size()) return 0.0f;
    return joints_[index].getCurrentValue();
}

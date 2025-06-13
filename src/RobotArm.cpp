#include "RobotArm.h"
#include <cstring>

RobotArm::RobotArm() {
    std::memset(jointAngles, 0, sizeof(jointAngles));
}

void RobotArm::setJointAngles(const double* angles) {
    std::memcpy(jointAngles, angles, sizeof(jointAngles));
}

void RobotArm::getJointAngles(double* outAngles) const {
    std::memcpy(outAngles, jointAngles, sizeof(jointAngles));
}

Pose RobotArm::forwardKinematics() {
    return fkSolver.solve(model, jointAngles);
}

bool RobotArm::inverseKinematics(const Pose& targetPose) {
    return ikSolver.solve(model, jointAngles, targetPose);
}

void RobotArm::setIKEpsilon(double eps) {
    ikSolver.setEpsilon(eps);
}

void RobotArm::setIKMaxIterations(int maxIter) {
    ikSolver.setMaxIterations(maxIter);
}

void RobotArm::setIKLearningRate(double lr) {
    ikSolver.setLearningRate(lr);
}

ArmModel& RobotArm::getModel() {
    return model;
}

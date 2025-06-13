#pragma once

#include "Pose.h"
#include "ArmModel.h"
#include "IKSolver.h"
#include "FKSolver.h"

class RobotArm {
public:
    RobotArm();

    void setJointAngles(const double* angles);
    void getJointAngles(double* outAngles) const;

    Pose forwardKinematics();
    bool inverseKinematics(const Pose& targetPose);

    // IK params
    void setIKEpsilon(double eps);
    void setIKMaxIterations(int maxIter);
    void setIKLearningRate(double lr);

    // Access to model
    ArmModel& getModel();

private:
    static constexpr int NUM_JOINTS = 6;
    double jointAngles[NUM_JOINTS];

    ArmModel model;
    IKSolver ikSolver;
    FKSolver fkSolver;
};

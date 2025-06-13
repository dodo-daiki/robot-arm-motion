#pragma once

#include "Pose.h"
#include "ArmModel.h"
#include <Eigen/Dense>

class IKSolver {
public:
    IKSolver();

    void setEpsilon(double eps);
    void setMaxIterations(int maxIter);
    void setLearningRate(double lr);

    bool solve(const ArmModel& model, double* jointAngles, const Pose& targetPose);

private:
    double epsilon;
    int maxIterations;
    double learningRate;
};

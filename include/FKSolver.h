#pragma once

#include "Pose.h"
#include "ArmModel.h"
#include <Eigen/Dense>

class FKSolver {
public:
    FKSolver();

    Pose solve(const ArmModel& model, const double* jointAngles);
};

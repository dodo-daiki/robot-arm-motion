#include "FKSolver.h"

FKSolver::FKSolver() {}

Pose FKSolver::solve(const ArmModel& model, const double* jointAngles) {
    Pose pose;

    // Dummy implementation: simple planar forward kinematics
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;

    for (int i = 0; i < model.getNumLinks(); ++i) {
        theta += jointAngles[i];
        x += model.getLinkLength(i) * cos(theta);
        y += model.getLinkLength(i) * sin(theta);
    }

    pose.position << x, y, 0;
    pose.orientation.setIdentity();
    return pose;
}

#pragma once
#include <vector>
#include "Joint.h"
#include "Link.h"
#include "KinematicsSolver.h"

class RobotArm {
public:
    RobotArm();

    void addJoint(float limit_min, float limit_max, float theta_offset);
    void addLink(float a, float alpha, float d);

    size_t getNumJoints() const;

    void computeIK(float target_x, float target_y, float target_z,
                   const std::vector<float>& initial_guess);

    float getJointAngle(size_t index) const;

private:
    std::vector<Joint> joints_;
    std::vector<Link> links_;
    KinematicsSolver solver_;
};

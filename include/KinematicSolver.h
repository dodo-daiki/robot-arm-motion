#pragma once
#include <vector>
#include "Joint.h"
#include "Link.h"
#include "Matrix4x4.h"

class KinematicsSolver {
public:
    KinematicsSolver(std::vector<Joint>& joints,
                     const std::vector<Link>& links);

    // 目標位置 (x,y,z)、初期推定theta、反復回数
    void computeIK(float target_x, float target_y, float target_z,
                   const std::vector<float>& initial_guess);

private:
    std::vector<Joint>& joints_;
    const std::vector<Link>& links_;

    void forwardKinematics(const std::vector<float>& joint_angles,
                           float& x, float& y, float& z) const;

    void computeJacobian(const std::vector<float>& joint_angles,
                         float J[3][3]) const;

    float clamp(float val, float min_val, float max_val) const;
};

#pragma once
#include <vector>
#include "Joint.h"
#include "Link.h"
#include "Matrix4x4.h"

class KinematicsSolver {
public:
    KinematicsSolver(std::vector<Joint>& joints,
                     const std::vector<Link>& links);

    // IKパラメータ設定
    void setMaxIterations(int max_iterations);
    void setTolerance(float tolerance);
    void setLearningRate(float learning_rate);

    void computeIK(float target_x, float target_y, float target_z,
                   float target_roll, float target_pitch, float target_yaw,
                   const std::vector<float>& initial_guess);

    void forwardKinematics(const std::vector<float>& joint_angles,
                           float& x, float& y, float& z,
                           float& roll, float& pitch, float& yaw) const;
private:
    std::vector<Joint>& joints_;
    const std::vector<Link>& links_;

    // 個別のパラメータ変数
    int max_iterations_;
    float tolerance_;
    float learning_rate_;

    

    void computeJacobian(const std::vector<float>& joint_angles,
                         std::vector<std::vector<float>>& J) const;

    float clamp(float val, float min_val, float max_val) const;
};

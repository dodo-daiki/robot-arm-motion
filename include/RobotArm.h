#pragma once
#include <vector>
#include "Joint.h"
#include "Link.h"
#include "Matrix4x4.h"
#include "KinematicsSolver.h"

class RobotArm {
public:
    RobotArm();

    void addJoint(float limit_min, float limit_max, float theta_offset);
    void addLink(float a, float alpha, float d);

    size_t getNumJoints() const;

    // IKパラメータ設定API
    void setMaxIterations(int max_iterations);
    void setTolerance(float tolerance);
    void setLearningRate(float learning_rate);

    // 6DOF対応版 computeIK
    void computeIK(float target_x, float target_y, float target_z,
               float target_roll, float target_pitch, float target_yaw);

    // ジョイント角度取得
    void setJointAngle(size_t index, float angle);

    float getJointAngle(size_t index) const;

private:
    std::vector<Joint> joints_;
    std::vector<Link> links_;
    KinematicsSolver solver_;
};

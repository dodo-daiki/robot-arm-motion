#pragma once

#include <Eigen/Dense>

struct Pose {
    Eigen::Vector3d position;       // x, y, z
    Eigen::Matrix3d orientation;    // 3x3 rotation matrix

    Pose() {
        position.setZero();
        orientation.setIdentity();
    }
};

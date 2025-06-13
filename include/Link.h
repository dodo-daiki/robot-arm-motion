#pragma once
#include <Eigen/Dense>

class Link {
public:
    Link(int id_, const Eigen::Vector3d& offset_, const Eigen::Matrix3d& rotation_);

    int getId() const;
    Eigen::Vector3d getOffset() const;
    void setOffset(const Eigen::Vector3d& offset);

    Eigen::Matrix3d getRotation() const;
    void setRotation(const Eigen::Matrix3d& rotation);

private:
    int id;
    Eigen::Vector3d offset;
    Eigen::Matrix3d rotation;
};

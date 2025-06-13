#pragma once
#include <Eigen/Dense>

class Joint {
public:
    Joint(double minLimit_, double maxLimit_, const Eigen::Vector3d& axis_);

    double getAngle() const;
    void setAngle(double angle_);

    double getMinLimit() const;
    double getMaxLimit() const;

    Eigen::Vector3d getAxis() const;
    void setAxis(const Eigen::Vector3d& axis_);

private:
    double angle;
    double minLimit;
    double maxLimit;
    Eigen::Vector3d axis;
};

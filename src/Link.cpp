#include "Link.h"

Link::Link(int id_, const Eigen::Vector3d& offset_, const Eigen::Matrix3d& rotation_)
    : id(id_), offset(offset_), rotation(rotation_) {}

int Link::getId() const {
    return id;
}

Eigen::Vector3d Link::getOffset() const {
    return offset;
}

void Link::setOffset(const Eigen::Vector3d& offset_) {
    offset = offset_;
}

Eigen::Matrix3d Link::getRotation() const {
    return rotation;
}

void Link::setRotation(const Eigen::Matrix3d& rotation_) {
    rotation = rotation_;
}

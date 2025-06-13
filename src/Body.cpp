#include "Body.h"

Body::Body() {}

void Body::addLink(const Link& link) {
    links.push_back(link);
}

void Body::addJoint(const Joint& joint) {
    joints.push_back(joint);
}

size_t Body::getNumLinks() const {
    return links.size();
}

size_t Body::getNumJoints() const {
    return joints.size();
}

const Link& Body::getLink(size_t index) const {
    return links.at(index);
}

Link& Body::getLink(size_t index) {
    return links.at(index);
}

const Joint& Body::getJoint(size_t index) const {
    return joints.at(index);
}

Joint& Body::getJoint(size_t index) {
    return joints.at(index);
}

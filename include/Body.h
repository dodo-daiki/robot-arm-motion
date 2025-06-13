#pragma once
#include <vector>
#include "Link.h"
#include "Joint.h"

class Body {
public:
    Body();

    void addLink(const Link& link);
    void addJoint(const Joint& joint);

    size_t getNumLinks() const;
    size_t getNumJoints() const;

    const Link& getLink(size_t index) const;
    Link& getLink(size_t index); // 可変アクセス用

    const Joint& getJoint(size_t index) const;
    Joint& getJoint(size_t index); // 可変アクセス用

private:
    std::vector<Link> links;
    std::vector<Joint> joints;
};

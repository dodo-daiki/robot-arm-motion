#include "Link.h"

Link::Link(float a, float alpha, float d)
    : a_(a), alpha_(alpha), d_(d) {}

float Link::getA() const {
    return a_;
}

float Link::getAlpha() const {
    return alpha_;
}

float Link::getD() const {
    return d_;
}

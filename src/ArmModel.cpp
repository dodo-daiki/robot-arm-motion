#include "ArmModel.h"

ArmModel::ArmModel() {
    for (int i = 0; i < MAX_LINKS; ++i) {
        linkLengths[i] = 0.1; // default 0.1 m
    }
}

void ArmModel::setLinkLength(int index, double length) {
    if (index >= 0 && index < MAX_LINKS) {
        linkLengths[index] = length;
    }
}

double ArmModel::getLinkLength(int index) const {
    if (index >= 0 && index < MAX_LINKS) {
        return linkLengths[index];
    }
    return 0.0;
}

int ArmModel::getNumLinks() const {
    return MAX_LINKS;
}

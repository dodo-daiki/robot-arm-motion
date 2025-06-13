#pragma once

class ArmModel {
public:
    ArmModel();

    void setLinkLength(int index, double length);
    double getLinkLength(int index) const;
    int getNumLinks() const;

private:
    static constexpr int MAX_LINKS = 6;
    double linkLengths[MAX_LINKS];
};

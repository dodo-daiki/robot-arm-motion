#pragma once

class Link {
public:
    Link(float a, float alpha, float d);

    float getA() const;
    float getAlpha() const;
    float getD() const;

private:
    float a_;
    float alpha_;
    float d_;
};

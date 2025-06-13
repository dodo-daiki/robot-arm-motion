#pragma once

class Joint {
public:
    Joint(float theta_offset, float limit_min, float limit_max);

    void setCurrentValue(float value);
    float getCurrentValue() const;

    float getThetaOffset() const;
    float getLimitMin() const;
    float getLimitMax() const;

    // valueをmin/max範囲にclampして返す
    float getClampedValue(float value) const;

private:
    float theta_offset_;
    float limit_min_;
    float limit_max_;
    float current_value_;

    float clamp(float val, float min_val, float max_val) const;
};

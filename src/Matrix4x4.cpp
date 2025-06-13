#include "Matrix4x4.h"
#include <math.h>

Matrix4x4::Matrix4x4() {
    setIdentity();
}

void Matrix4x4::setIdentity() {
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            data[r][c] = (r == c) ? 1.0f : 0.0f;
        }
    }
}

Matrix4x4 Matrix4x4::operator*(const Matrix4x4& other) const {
    Matrix4x4 result;
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            result.data[r][c] = 0.0f;
            for (int k = 0; k < 4; ++k) {
                result.data[r][c] += data[r][k] * other.data[k][c];
            }
        }
    }
    return result;
}

Matrix4x4& Matrix4x4::operator*=(const Matrix4x4& other) {
    *this = *this * other;
    return *this;
}

void Matrix4x4::copyFrom(const Matrix4x4& other) {
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            data[r][c] = other.data[r][c];
        }
    }
}

void Matrix4x4::getRPY(float& roll, float& pitch, float& yaw) const {
    // Rotation部分は data[0..2][0..2]
    // ZYX順 (yaw-pitch-roll) でR,P,Yを取得

    float sy = sqrt(data[0][0] * data[0][0] + data[1][0] * data[1][0]);

    bool singular = sy < 1e-6;

    if (!singular) {
        pitch = atan2(-data[2][0], sy);
        roll  = atan2(data[2][1], data[2][2]);
        yaw   = atan2(data[1][0], data[0][0]);
    } else {
        pitch = atan2(-data[2][0], sy);
        roll  = atan2(-data[1][2], data[1][1]);
        yaw   = 0;
    }
}

void Matrix4x4::print() const {
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            Serial.print(data[r][c], 6);  // 小数点以下6桁表示
            Serial.print("\t");
        }
        Serial.println();
    }
    Serial.println();
}

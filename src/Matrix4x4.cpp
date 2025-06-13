#include "Matrix4x4.h"

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

void Matrix4x4::multiply(const Matrix4x4& other) {
    float result[4][4];

    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            result[r][c] = 0.0f;
            for (int k = 0; k < 4; ++k) {
                result[r][c] += data[r][k] * other.data[k][c];
            }
        }
    }

    // 結果を自分自身にコピー
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            data[r][c] = result[r][c];
        }
    }
}

void Matrix4x4::copyFrom(const Matrix4x4& other) {
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            data[r][c] = other.data[r][c];
        }
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

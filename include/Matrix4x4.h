#pragma once
#include <Arduino.h>

class Matrix4x4 {
public:
    Matrix4x4();                 // コンストラクタ → 単位行列で初期化

    void setIdentity();          // 単位行列にセット

    // 演算子オーバーロード
    Matrix4x4 operator*(const Matrix4x4& other) const;
    Matrix4x4& operator*=(const Matrix4x4& other);

    void copyFrom(const Matrix4x4& other);

    void getRPY(float& roll, float& pitch, float& yaw) const;  // Roll-Pitch-Yawを取得

    void print() const;          // デバッグ用出力

    float data[4][4];            // データ本体
};
